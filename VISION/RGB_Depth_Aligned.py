#!/usr/bin/env python3

import cv2
import depthai as dai
import math
import numpy as np

# Weights to use when blending rgb/depth image
rgbWeight = 0.2
depthWeight = 0.8

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
camRgb = pipeline.create(dai.node.ColorCamera)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutSpatialData = pipeline.create(dai.node.XLinkOut)
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")
xoutRgb.setStreamName("rgb")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
spatialLocationCalculator.inputConfig.setWaitForMessage(False)

camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setFps(30)
camRgb.setIspScale(2, 3)  # downscale to 720p

# Create a fine mesh ROIs, neglecting the first and last two rows and columns
numRows = 30
numCols = 30
for row in range(5, numRows - 5):
    for col in range(2, numCols - 2):
        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = 200
        config.depthThresholds.upperThreshold = 10000
        topLeft = dai.Point2f(col * (1.0 / numCols), row * (1.0 / numRows))
        bottomRight = dai.Point2f((col + 1) * (1.0 / numCols), (row + 1) * (1.0 / numRows))
        config.roi = dai.Rect(topLeft, bottomRight)
        spatialLocationCalculator.initialConfig.addROI(config)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)
spatialLocationCalculator.out.link(xoutSpatialData.input)
xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)
camRgb.isp.link(xoutRgb.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    device.setIrLaserDotProjectorIntensity(1000)  # Use the updated method

    # Output queues will be used to get the frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
    rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    color = (0, 200, 40)
    fontType = cv2.FONT_HERSHEY_TRIPLEX

    while True:
        inDepth = depthQueue.get()  # Blocking call, will wait until new data arrives
        inRgb = rgbQueue.get()  # Blocking call, will wait until new data arrives

        depthFrame = inDepth.getFrame()  # depthFrame values are in millimeters
        rgbFrame = inRgb.getCvFrame()

        # Depth frame processing
        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        spatialData = spatialCalcQueue.get().getSpatialLocations()
        rois_with_distances = []

        for depthData in spatialData:
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            coords = depthData.spatialCoordinates
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)
            if distance > 400:  # Filter out distances between 0 and 40 cm
                rois_with_distances.append((distance, roi, xmin, ymin, xmax, ymax))

        # Find the 4 closest ROIs if available
        if rois_with_distances:
            rois_with_distances.sort(key=lambda x: x[0])
            closest_rois = rois_with_distances[:1]
            mean_x = int(np.mean([roi[2] + (roi[4] - roi[2]) / 2 for roi in closest_rois]))
            mean_y = int(np.mean([roi[3] + (roi[5] - roi[3]) / 2 for roi in closest_rois]))

            for distance, roi, xmin, ymin, xmax, ymax in closest_rois:
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, thickness=1)
                cv2.putText(depthFrameColor, "{:.1f}cm".format(distance / 10), (xmin + 5, ymin + 15), fontType, 0.3, color)
            mean_depth_text = "Mean Depth: {:.1f}cm".format(np.mean([d[0] for d in closest_rois]) / 10)
        else:
            mean_x, mean_y = -1, -1
            mean_depth_text = "No ROI > 40cm"

        # Blend RGB and depth frames
        if len(depthFrameColor.shape) < 3:
            depthFrameColor = cv2.cvtColor(depthFrameColor, cv2.COLOR_GRAY2BGR)
        blended = cv2.addWeighted(rgbFrame, rgbWeight, depthFrameColor, depthWeight, 0)

        # Display mean depth on the blended image in white
        cv2.putText(blended, mean_depth_text, (blended.shape[1] - 300, 30), fontType, 0.6, (255, 255, 255))

        # Display the mean position as a dot
        if mean_x >= 0 and mean_y >= 0:
            cv2.circle(blended, (mean_x, mean_y), 5, (255, 255, 255), -1)

        # Resize frames with factor 0.25
        rgbFrame_resized = cv2.resize(rgbFrame, (0, 0), fx=0.25, fy=0.25)
        depthFrameColor_resized = cv2.resize(depthFrameColor, (0, 0), fx=0.25, fy=0.25)
        blended_resized = cv2.resize(blended, (0, 0), fx=0.25, fy=0.25)

        # Show frames (comment out if not needed)
        cv2.imshow("rgb", rgbFrame_resized)
        cv2.imshow("depth", depthFrameColor_resized)
        cv2.imshow("rgb-depth", blended)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
