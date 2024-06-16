import cv2
import depthai as dai
import math
import numpy as np
from functions import order_points, click_event, draw_grid_within_rectangle, point_in_polygon, draw_highlighted_rois, interpolate_position_within_rectangle

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
monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
spatialLocationCalculator.inputConfig.setWaitForMessage(False)

# Create 400 ROIs for a 25x25 grid
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

# Linking depth pipeline
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

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
    ordered_corners = None  # Initialize ordered_corners
    highlighted_rois = []  # List to store highlighted ROIs

    # Assume an offset between depth and RGB image
    depth_to_rgb_offset_x = 10
    depth_to_rgb_offset_y = 10

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

        spatialData = spatialCalcQueue.get().getSpatialLocations()
        distances = []
        highlighted_rois.clear()  # Clear the highlighted ROIs list
        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            # Skip the first and last columns and the first and last rows
            if xmin < depthFrameColor.shape[1] / num_cols or xmax > depthFrameColor.shape[1] * (num_cols - 1) / num_cols:
                continue
            if ymin < depthFrameColor.shape[0] / num_rows or ymax > depthFrameColor.shape[0] * (num_rows - 1) / num_rows:
                continue

            coords = depthData.spatialCoordinates
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)

            # Filter out distances that are 0.0m or outside the range 0.3m to 0.8m
            if distance == 0.0 or distance < 300 or distance > 800:
                continue

            distances.append((distance, (xmin, ymin, xmax, ymax)))

        # Highlight the 2 ROIs with the lowest distances
        distances.sort(key=lambda x: x[0])
        for i, (distance, (xmin, ymin, xmax, ymax)) in enumerate(distances[:2]):
            currentColor = highlightColor if i < 2 else color
            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), currentColor, thickness=2)
            cv2.putText(depthFrameColor, "{:.1f}m".format(distance / 1000), (xmin + 10, ymin + 20), fontType, 0.6, currentColor)
            highlighted_rois.append((xmin, ymin, xmax, ymax))

        # Map the highlighted ROIs to the video frame dimensions
        frame_height, frame_width = frame.shape[:2]
        depth_height, depth_width = depthFrameColor.shape[:2]
        scaled_highlighted_rois = [
            (
                int(xmin * frame_width / depth_width) + depth_to_rgb_offset_x,
                int(ymin * frame_height / depth_height) + depth_to_rgb_offset_y,
                int(xmax * frame_width / depth_width) + depth_to_rgb_offset_x,
                int(ymax * frame_height / depth_height) + depth_to_rgb_offset_y
            ) for (xmin, ymin, xmax, ymax) in highlighted_rois
        ]

        if cv2.waitKey(1) == ord('q'):
            break

        # Show the frames
        cv2.imshow("depth", depthFrameColor)
        draw_highlighted_rois(frame, scaled_highlighted_rois, highlightColor)
        cv2.imshow('Detected Markers and Grid', frame)
        cv2.setMouseCallback('Detected Markers and Grid', click_event, (frame, clicked_corners, corner_names))

    cv2.destroyAllWindows()
