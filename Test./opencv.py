import cv2 as cv
import numpy as np


video_path = "Path"
output_file = 'grayscale_video.mp4'


# --- VIDEO INPUT ---
cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
size = (width, height)
 
# --- VIDEO WRITERS ---
bgr_writer = cv2.VideoWriter(
    os.path.join(output_folder, 'bgr_saved.mp4'),
    cv2.VideoWriter_fourcc(*'mp4v'),
    fps, size)
rgb_writer = cv2.VideoWriter(
    os.path.join(output_folder, 'rgb_saved.mp4'),
    cv2.VideoWriter_fourcc(*'mp4v'),
    fps, size)
 
# --- PROCESS FRAMES ---
while True:
    ret, frame = cap.read()
    if not ret:
        break
 
    # Save the original BGR frame
    bgr_writer.write(frame)
 
    # Convert frame to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
 
    # Save the RGB frame
    rgb_writer.write(rgb_frame)

    cap = cv2.VideoCapture(video_path)
fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
size = (width, height)
 
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, fps, size, isColor=True)
 
while True:
    ret, frame = cap.read()
    if not ret:
        break
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    out.write(hsv_frame)  # Save HSV frame directly

im = cv.imread('test.jpg')
assert im is not None, "file could not be read, check with os.path.exists()"
imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 255, 0)
im2, contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

 
# --- CLEAN UP ---
cap.release()
bgr_writer.release()
rgb_writer.release()
print("Videos saved successfully in:", output_folder)

cv.destroyAllWindows()