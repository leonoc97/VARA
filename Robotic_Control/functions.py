# functions.py

import numpy as np
import cv2

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect

def click_event(event, x, y, flags, param):
    frame, clicked_corners, corner_names = param
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_corners.append((x, y))
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        if len(clicked_corners) <= 4:
            cv2.putText(frame, corner_names[len(clicked_corners) - 1], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

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

def draw_highlighted_rois(frame, rois, color):
    for (xmin, ymin, xmax, ymax) in rois:
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, thickness=2)
        cv2.putText(frame, "ROI", (xmin + 10, ymin + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color)

def interpolate_position_within_rectangle(point, rect):
    (top_left, top_right, bottom_right, bottom_left) = rect
    width = np.linalg.norm(top_right - top_left)
    height = np.linalg.norm(top_left - bottom_left)

    # Calculate position relative to top-left corner
    vector_top = top_right - top_left
    vector_left = bottom_left - top_left

    relative_position = point - top_left

    x_position = np.dot(relative_position, vector_top) / width
    y_position = np.dot(relative_position, vector_left) / height

    return x_position, y_position
