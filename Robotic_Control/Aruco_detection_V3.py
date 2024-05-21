import cv2
from cv2 import aruco
import depthai as dai
import numpy as np
from ArucoDetection_definitions import draw_field

def get_markers(vid_frame, aruco_dictionary, aruco_parameters):
    bboxs, ids, rejected = cv2.aruco.detectMarkers(vid_frame, aruco_dictionary, parameters=aruco_parameters)
    if ids is not None:
        ids_sorted = [id_number[0] for id_number in ids]
    else:
        ids_sorted = ids
    return bboxs, ids_sorted

def main():
    # Dictionary to specify the type of the marker
    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    # Detector parameters for ArUco markers
    param_markers = aruco.DetectorParameters()

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    cam_rgb = pipeline.createColorCamera()
    xout_video = pipeline.createXLinkOut()

    xout_video.setStreamName("video")

    # Properties
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

    # Linking
    cam_rgb.preview.link(xout_video.input)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        # Get data queues
        video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

        while True:
            video_frame = video_queue.get()
            frame = video_frame.getCvFrame()

            # Detect 4x4 ArUco markers in the video frame
            markers, ids = get_markers(frame, marker_dict, param_markers)

            # Draw detected markers on the frame
            if markers:
                cv2.aruco.drawDetectedMarkers(frame, markers)

            # Draw field based on markers
            frame_with_field, square_found = draw_field(frame, markers, ids)

            # Display the frame with the field
            cv2.imshow("frame_with_field", frame_with_field)

            # Check for 'q' key press to exit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    # Close down the video stream
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
