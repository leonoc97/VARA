#!/usr/bin/env python3

import cv2
import depthai as dai
import math
import numpy as np
from functions import order_points, click_event

# Create pipeline
pipeline = dai.Pipeline()

# Define depth sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# Define RGB sources and outputs
camRgb = pipeline.createColorCamera()
camRgb.setPreviewSize(640, 480)
camRgb.setInterleaved(False)

xoutVideo = pipeline.createXLinkOut()
xoutVideo.setStreamName("video")

camRgb.preview.link(xoutVideo.input)

# Properties for depth
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
spatialLocationCalculator.inputConfig.setWaitForMessage(False)

# Create 400 ROIs for a 20x20 grid
num_rows = 20
num_cols = 20
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

# Linking depth pipeline
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
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


def draw_grid_within_rectangle(frame, ordered_corners, grid_spacing_px):
    (top_left, top_right, bottom_right, bottom_left) = ordered_corners
    width = np.linalg.norm(top_right - top_left)
    height = np.linalg.norm(top_left - bottom_left)
    num_vertical_lines = int(width // grid_spacing_px)
    num_horizontal_lines = int(height // grid_spacing_px)

    for i in range(1, num_vertical_lines):
        start = top_left + i * (top_right - top_left) / num_vertical_lines
        end = bottom_left + i * (bottom_right - bottom_left) / num_vertical_lines
        cv2.line(frame, tuple(start.astype(int)), tuple(end.astype(int)), (255, 255, 255), 1)

    for i in range(1, num_horizontal_lines):
        start = top_left + i * (bottom_left - top_left) / num_horizontal_lines
        end = top_right + i * (bottom_right - top_right) / num_horizontal_lines
        cv2.line(frame, tuple(start.astype(int)), tuple(end.astype(int)), (255, 255, 255), 1)


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    p1x, p1y = polygon[0]
    for i in range(len(polygon) + 1):
        p2x, p2y = polygon[i % len(polygon)]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    device.setIrLaserDotProjectorBrightness(1000)

    # Output queue will be used to get the depth frames and video frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    videoQueue = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    color = (0, 200, 40)
    highlightColor = (0, 0, 255)  # Color for highlighting the lowest distance ROIs
    fontType = cv2.FONT_HERSHEY_TRIPLEX

    while True:
        inDepth = depthQueue.get()  # Blocking call, will wait until a new data has arrived
        inFrame = videoQueue.get()
        frame = inFrame.getCvFrame()

        depthFrame = inDepth.getFrame()  # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

        if markerIds is not None:
            marker_corners = [corner.reshape((4, 2)) for corner in markerCorners]
            for corners in marker_corners:
                cv2.polylines(frame, [np.int32(corners)], True, (0, 255, 0), 2)

            if len(marker_corners) == 4:
                all_corners = np.vstack(marker_corners)
                ordered_corners = order_points(all_corners)
                cv2.polylines(frame, [np.int32(ordered_corners)], True, (0, 0, 255), 2)
                draw_grid_within_rectangle(frame, ordered_corners, grid_spacing_px)

        spatialData = spatialCalcQueue.get().getSpatialLocations()
        distances = []
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            # Skip the first and last columns and the first and last rows
            if xmin < depthFrameColor.shape[1] / num_cols or xmax > depthFrameColor.shape[1] * (
                    num_cols - 1) / num_cols:
                continue
            if ymin < depthFrameColor.shape[0] / num_rows or ymax > depthFrameColor.shape[0] * (
                    num_rows - 1) / num_rows:
                continue

            coords = depthData.spatialCoordinates
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)

            # Filter out distances that are 0.0m or outside the range 0.3m to 0.8m
            if distance == 0.0 or distance < 300 or distance > 800:
                continue

            if point_in_polygon(((xmin + xmax) / 2, (ymin + ymax) / 2), ordered_corners):
                distances.append((distance, (xmin, ymin, xmax, ymax)))

        # Highlight the 4 ROIs with the lowest distances
        distances.sort(key=lambda x: x[0])
        for i, (distance, (xmin, ymin, xmax, ymax)) in enumerate(distances[:4]):
            currentColor = highlightColor if i < 4 else color
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), currentColor, thickness=2)
            cv2.putText(depthFrameColor, "{:.1f}m".format(distance / 1000), (xmin + 10, ymin + 20), fontType, 0.6,
                        currentColor)

        # Show the frame
        cv2.imshow("depth", depthFrameColor)
        cv2.imshow('Detected Markers and Grid', frame)
        cv2.setMouseCallback('Detected Markers and Grid', click_event, (frame, clicked_corners, corner_names))

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
