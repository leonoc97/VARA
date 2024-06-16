import cv2 as cv
import numpy as np
from functions import order_points, click_event
import depthai as dai
def create_pipeline():
    """
    Set up and return the DepthAI pipeline.
    """
    pipeline = dai.Pipeline()
    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(640, 480)
    camRgb.setInterleaved(False)

    xoutVideo = pipeline.createXLinkOut()
    xoutVideo.setStreamName("video")

    camRgb.preview.link(xoutVideo.input)

    return pipeline

def draw_grid_within_rectangle(frame, ordered_corners, grid_spacing_px):
    (top_left, top_right, bottom_right, bottom_left) = ordered_corners
    width = np.linalg.norm(top_right - top_left)
    height = np.linalg.norm(top_left - bottom_left)
    num_vertical_lines = int(width // grid_spacing_px)
    num_horizontal_lines = int(height // grid_spacing_px)

    for i in range(1, num_vertical_lines):
        start = top_left + i * (top_right - top_left) / num_vertical_lines
        end = bottom_left + i * (bottom_right - bottom_left) / num_vertical_lines
        cv.line(frame, tuple(start.astype(int)), tuple(end.astype(int)), (255, 255, 255), 1)

    for i in range(1, num_horizontal_lines):
        start = top_left + i * (bottom_left - top_left) / num_horizontal_lines
        end = top_right + i * (bottom_right - top_right) / num_horizontal_lines
        cv.line(frame, tuple(start.astype(int)), tuple(end.astype(int)), (255, 255, 255), 1)

def main():
    pipeline = create_pipeline()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    clicked_corners = []
    corner_names = ["top-left", "top-right", "bottom-left", "bottom-right"]
    marker_corners = []
    grid_spacing_cm = 5
    cm_to_pixel = 10
    grid_spacing_px = grid_spacing_cm * cm_to_pixel

    with dai.Device(pipeline) as device:
        videoQueue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

        while True:
            inFrame = videoQueue.get()
            frame = inFrame.getCvFrame()
            markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

            if markerIds is not None:
                marker_corners = [corner.reshape((4, 2)) for corner in markerCorners]
                for corners in marker_corners:
                    cv.polylines(frame, [np.int32(corners)], True, (0, 255, 0), 2)

                if len(marker_corners) == 4:
                    all_corners = np.vstack(marker_corners)
                    ordered_corners = order_points(all_corners)
                    cv.polylines(frame, [np.int32(ordered_corners)], True, (0, 0, 255), 2)
                    draw_grid_within_rectangle(frame, ordered_corners, grid_spacing_px)

            cv.imshow('Detected Markers and Grid', frame)
            cv.setMouseCallback('Detected Markers and Grid', click_event, (frame, clicked_corners, corner_names))

            if cv.waitKey(1) == ord('q'):
                break

    cv.destroyAllWindows()

if __name__ == "__main__":
    main()