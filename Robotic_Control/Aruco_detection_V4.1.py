#!/usr/bin/env python3

import cv2
import depthai as dai
import math
import numpy as np
from functions import order_points, click_event, draw_grid_within_rectangle, point_in_polygon, draw_highlighted_rois, interpolate_position_within_rectangle

# Weights to use when blending depth/rgb image (should equal 1.0)
rgbWeight = 0.4
depthWeight = 0.6

def updateBlendWeights(percent_rgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image

    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percent_rgb) / 100.0
    depthWeight = 1.0 - rgbWeight

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

# Create 400 ROIs for a 20x20 grid
num_rows = 25
num_cols = 25
for i in range(num_rows):
    for j in range(num_cols):
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = 200
        config.depthThresholds.upperThreshold = 10000
        x_start = j / num_cols
        x_end = (j + 1) / num_cols
        y_start = i / num_rows
        y_end = (i + 1) / num_rows
        config.roi = dai.Rect(dai.Point2f(x_start, y_start), dai.Point2f(x_end, y_end))
        spatialLocationCalculator.initialConfig.addROI(config)


xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)
rgbOut = pipeline.create(dai.node.XLinkOut)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")
rgbOut.setStreamName("rgb")

# Properties
rgbCamSocket = dai.CameraBoardSocket.CAM_A
fps = 30
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

camRgb.setBoardSocket(rgbCamSocket)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
camRgb.setFps(fps)

# For now, RGB needs fixed focus to properly align with depth.
# This value was used during calibration
device = dai.Device()
try:
    calibData = device.readCalibration2()
    lensPosition = calibData.getLensPosition(rgbCamSocket)
    if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
except:
    raise

left.setResolution(monoResolution)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
left.setFps(fps)
right.setResolution(monoResolution)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
right.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

# Linking
camRgb.video.link(rgbOut.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

# ArUco marker detection setup
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
clicked_corners = []
corner_names = ["top-left", "top-right", "bottom-left", "bottom-right"]
marker_corners = []
grid_spacing_cm = 5
cm_to_pixel = 10
grid_spacing_px = grid_spacing_cm * cm_to_pixel

# Known distances between markers in centimeters
horizontal_distance_cm = 32
vertical_distance_cm = 28

# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)

    frameRgb = None
    frameDisp = None

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    rgbWindowName = "rgb"
    depthWindowName = "depth"
    blendedWindowName = "rgb-depth"
    cv2.namedWindow(rgbWindowName)
    cv2.namedWindow(depthWindowName)
    cv2.namedWindow(blendedWindowName)
    cv2.createTrackbar('RGB Weight %', blendedWindowName, int(rgbWeight * 100), 100, updateBlendWeights)

    while True:
        latestPacket = {}
        latestPacket["rgb"] = None
        latestPacket["depth"] = None

        queueEvents = device.getQueueEvents(("rgb", "depth"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["rgb"] is not None:
            frameRgb = latestPacket["rgb"].getCvFrame()
            cv2.imshow(rgbWindowName, frameRgb)

        if latestPacket["depth"] is not None:
            frameDisp = latestPacket["depth"].getFrame()
            maxDisparity = stereo.initialConfig.getMaxDisparity()
            # Optional, extend range 0..95 -> 0..255, for a better visualization
            if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
            # Optional, apply false colorization
            if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_HOT)
            frameDisp = np.ascontiguousarray(frameDisp)
            cv2.imshow(depthWindowName, frameDisp)

        # Blend when both received
        if frameRgb is not None and frameDisp is not None:
            # Need to have both frames in BGR format before blending
            if len(frameDisp.shape) < 3:
                frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(frameRgb, rgbWeight, frameDisp, depthWeight, 0)
            cv2.imshow(blendedWindowName, blended)
            frameRgb = None
            frameDisp = None

        if cv2.waitKey(1) == ord('q'):
            break

        # Detect ArUco markers and process frames
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frameRgb)
        if markerIds is not None:
            marker_corners = [corner.reshape((4, 2)) for corner in markerCorners]
            for corners in marker_corners:
                cv2.polylines(frameRgb, [np.int32(corners)], True, (0, 255, 0), 2)

            if len(marker_corners) == 4:
                all_corners = np.vstack(marker_corners)
                ordered_corners = order_points(all_corners)
                cv2.polylines(frameRgb, [np.int32(ordered_corners)], True, (0, 0, 255), 2)
                draw_grid_within_rectangle(frameRgb, ordered_corners, grid_spacing_px)

        spatialData = latestPacket["depth"].getSpatialLocations() if latestPacket["depth"] else []
        distances = []
        highlighted_rois = []  # Clear the highlighted ROIs list
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=frameDisp.shape[1], height=frameDisp.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            # Skip the first and last columns and the first and last rows
            if xmin < frameDisp.shape[1] / num_cols or xmax > frameDisp.shape[1] * (num_cols - 1) / num_cols:
                continue
            if ymin < frameDisp.shape[0] / num_rows or ymax > frameDisp.shape[0] * (num_rows - 1) / num_rows:
                continue

            coords = depthData.spatialCoordinates
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)

            # Filter out distances that are 0.0m or outside the range 0.3m to 0.8m
            if distance == 0.0 or distance < 300 or distance > 800:
                continue

            if ordered_corners is not None and point_in_polygon(((xmin + xmax) / 2, (ymin + ymax) / 2), ordered_corners):
                distances.append((distance, (xmin, ymin, xmax, ymax)))

        # Highlight the 2 ROIs with the lowest distances
        distances.sort(key=lambda x: x[0])
        for i, (distance, (xmin, ymin, xmax, ymax)) in enumerate(distances[:1]):
            currentColor = highlightColor if i < 2 else color
            cv2.rectangle(frameDisp, (xmin, ymin), (xmax, ymax), currentColor, thickness=2)
            cv2.putText(frameDisp, "{:.1f}m".format(distance / 1000), (xmin + 10, ymin + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.6, currentColor)
            highlighted_rois.append((xmin, ymin, xmax, ymax))

        # Map the highlighted ROIs to the video frame dimensions
        frame_height, frame_width = frameRgb.shape[:2]
        depth_height, depth_width = frameDisp.shape[:2]
        scaled_highlighted_rois = [
            (
                int(xmin * frame_width / depth_width),
                int(ymin * frame_height / depth_height),
                int(xmax * frame_width / depth_width),
                int(ymax * frame_height / depth_height)
            ) for (xmin, ymin, xmax, ymax) in highlighted_rois
        ]
        if ordered_corners is not None:
            top_left_marker_corner = ordered_corners[0]  # Top-left corner of the bounding rectangle

            # Calculate scaling factors
            horizontal_scale = horizontal_distance_cm / (ordered_corners[1][0] - ordered_corners[0][0])
            vertical_scale = vertical_distance_cm / (ordered_corners[2][1] - ordered_corners[0][1])

            for roi in scaled_highlighted_rois:
                xmin, ymin, xmax, ymax = roi
                center_point = ((xmin + xmax) / 2, (ymin + ymax) / 2)
                adjusted_center_point = (center_point[0] - top_left_marker_corner[0], center_point[1] - top_left_marker_corner[1])

                # Calculate distances in centimeters
                x_distance_cm = adjusted_center_point[0] * horizontal_scale
                y_distance_cm = adjusted_center_point[1] * vertical_scale

                print(f"Relative position from top-left marker: x={x_distance_cm:.2f} cm, y={y_distance_cm:.2f} cm")

                # Draw lines from top-left ArUco marker to the center of ROI
                cv2.line(frameRgb, (int(top_left_marker_corner[0]), int(top_left_marker_corner[1])),
                         (int(center_point[0]), int(center_point[1])), (255, 0, 0), 1)
                cv2.putText(frameRgb, f"x={x_distance_cm:.2f}cm", (int(center_point[0]), int(center_point[1]) - 10),
                            cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0))
                cv2.putText(frameRgb, f"y={y_distance_cm:.2f}cm", (int(center_point[0]), int(center_point[1]) + 10),
                            cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0))

        if cv2.waitKey(1) == ord('q'):
            break

        # Show the frames
        cv2.imshow("depth", frameDisp)
        draw_highlighted_rois(frameRgb, scaled_highlighted_rois, highlightColor)
        cv2.imshow('Detected Markers and Grid', frameRgb)
        cv2.setMouseCallback('Detected Markers and Grid', click_event, (frameRgb, clicked_corners, corner_names))

    cv2.destroyAllWindows()
