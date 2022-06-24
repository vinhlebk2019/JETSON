

# this is default file of source code, at the begining, research this code

import matplotlib.pyplot as plt
import numpy as np
from scipy.misc import imresize
from moviepy.editor import VideoFileClip
from IPython.display import HTML
from keras.models import load_model
import tensorflow as tf
from cv2 import cv2
import time
model = load_model('full_CNN_model.h5')


# Class to average lanes with
class Lanes:
    def __init__(self):
        self.recent_fit = []
        self.avg_fit = []


video_name = "video/VAS/vid_1.mp4"
average = 20


def road_lines(input_image):
    """ Takes in a road image, re-sizes for the model,
    predicts the lane to be drawn from the model in G color,
    recreates an RGB image of a lane and merges with the
    original road image.
    """
    # Get image ready for feeding into model
    small_img = imresize(input_image, (80, 160, 3))
    small_img = np.array(small_img)
    small_img = small_img[None, :, :, :]

    # Make prediction with neural network (un-normalize value by multiplying by 255)
    prediction = model.predict(small_img[:60, :, :])[0] * 255

    # Add lane prediction to list for averaging
    lanes.recent_fit.append(prediction)
    # Only using last five for average
    if len(lanes.recent_fit) > average:
        lanes.recent_fit = lanes.recent_fit[1:]

    # Calculate average detection
    lanes.avg_fit = np.mean(np.array([i for i in lanes.recent_fit]), axis=0)

    # Generate fake R & B color dimensions, stack with G
    blanks = np.zeros_like(lanes.avg_fit).astype(np.uint8)
    lane_drawn = np.dstack((blanks, lanes.avg_fit, blanks))

    # Re-size to match the original image
    lane_image = imresize(lane_drawn, (height, width, 3))

    # Merge the lane drawing onto the original image
    result = cv2.addWeighted(input_image, 1, lane_image, 1.0, 0.0)
    return lane_image, result

lanes = Lanes()
# Read lane realtime from webcam
cap = cv2.VideoCapture(0)
# default frame in BGR channels
start_frame = time.time()
i = 0
while cap.isOpened():
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))
    if ret:
        pre_out_img, output_img = road_lines(frame)
        cv2.imshow("Result", output_img)
        stop_frame = time.time()
        FPS = 1 / (stop_frame - start_frame)
        print(FPS)
        start_frame = stop_frame
        if cv2.waitKey(1) & 0xFF == ord('d'):
            break
    else:
        break
cap.release()
cv2.destroyAllWindows()
