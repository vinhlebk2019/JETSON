import os
import numpy as np
#from scipy.misc import imresize
from moviepy.editor import VideoFileClip
from IPython.display import HTML

from tensorflow import keras

from tensorflow.keras.models import load_model
import tensorflow as tf

#from cv2 import cv2
import cv2
import time
from PIL import Image, ImageEnhance

# Class to average lanes with
class Lanes:
    def __init__(self):
        self.recent_fit = []
        self.avg_fit = []

class out_paras:
    def __init__(self):
        self.radius = []
        self.offset = []
        self.msg = ""
        self.error_msg = ""
        self.pre_radius = None
        self.pre_offset = []

########## Parameters ##########
# Lane parameter
lanes = Lanes()
# Load model
model = load_model('full_CNN_model.h5')
# all output data
output_data = out_paras() 
# continue_detect_flag
con_detect_flag = True
# Tinh ra don vi meter tren pixel
height_to_meter = 30
width_to_meter = 2.7
# kich thuoc anh ngo ra
image_out_width = 700
image_out_height = 400
# Prediction avarage
average = 7

def predict_lane(input_image):
    """ Takes in a road image, re-sizes for the model,
    predicts the lane to be drawn from the model in G color,
    recreates an RGB image of a lane and merges with the
    original road image.
    """
    height = input_image.shape[0]
    width = input_image.shape[1]

    # Get image ready for feeding into model
    small_img = cv2.resize(input_image, (160, 80))
    small_img = np.array(small_img)
    small_img = small_img[None, :, :, :]

    # Make prediction with neural network (un-normalize value by multiplying by 255)
    prediction = model.predict(small_img[:, :, :])[0] * 255

    # Add lane prediction to list for averaging
    lanes.recent_fit.append(prediction)
    # Only using last five for average
    if len(lanes.recent_fit) > average:
        lanes.recent_fit = lanes.recent_fit[1:]

    # Calculate average detection
    lanes.avg_fit = np.mean(np.array([i for i in lanes.recent_fit]), axis=0)

    # Generate fake R & B color dimensions, stack with G
    blanks = np.zeros_like(lanes.avg_fit).astype(np.uint8)
    draw_predict = np.dstack((blanks, lanes.avg_fit, blanks))

    # Re-size to match the original image
    draw_predict = cv2.resize(draw_predict, (width, height))
    input_image = cv2.resize(input_image, (width, height))

    draw_predict = draw_predict.astype(np.uint8)
    input_image = input_image.astype(np.uint8)

    #print (input_image.shape)
    #print (draw_predict.shape)

    # Merge the lane drawing onto the original image
    out_predict = cv2.addWeighted(input_image, 1, draw_predict, 1, 0.0)
    return draw_predict, out_predict


def cal_curvature_and_offset(left_fit, right_fit, last_pt_center, size):
    height = size[0]
    width = size[1]

    ym_per_pix = height_to_meter / height
    xm_per_pix = width_to_meter / width

    # ???
    y_eval_left = height  # example 720p video/image, so last (lowest on screen) y index is 719
    y_eval_right = height

    left_x = left_fit[1]
    left_y = left_fit[0]

    right_x = right_fit[1]
    right_y = right_fit[0]

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(left_y * ym_per_pix, left_x * xm_per_pix, 2)
    right_fit_cr = np.polyfit(right_y * ym_per_pix, right_x * xm_per_pix, 2)

    # Compute R_curve (radius of curvature)
    left_curveR = ((1 + (2 * left_fit_cr[0] * y_eval_left * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
        2 * left_fit_cr[0])
    right_curveR = ((1 + (
            2 * right_fit_cr[0] * y_eval_right * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
        2 * right_fit_cr[0])
    output_data.radius = (left_curveR + right_curveR) // 2

    # chia 10^16 de de quan sat, km dung thi chia 1000, m thi khong can chia
    output_data.radius = output_data.radius / 10000000000000000

    # Kiem tra loi bang cach sua doan kenh ben duoi hoac thay bang doan code khac
    # if output_data.pre_radius is None:
    #     output_data.pre_radius = output_data.radius
    # else:
    #     if (output_data.radius // output_data.pre_radius > 3) | (output_data.pre_radius // output_data.radius > 3):
    #         output_data.error_msg = "Bad calculation"
    #         output_data.msg = output_data.msg + output_data.error_msg
    #         output_data.radius = output_data.pre_radius
    #     else:
    #         output_data.pre_radius = output_data.radius
    #         output_data.error_msg = "Good calculation"
    #         output_data.msg = output_data.msg + output_data.error_msg

    # Calculate Center offset
    output_data.offset = (last_pt_center - (width // 2)) * xm_per_pix

    lat_msg = "Car is {:.2f} m to the center...".format(output_data.offset)
    curvature_msg = "Curvature = {:.2f} km".format(output_data.radius)
    output_data.msg = lat_msg + curvature_msg


def edge_Canny_lanes(predict_area):
    green_channel = predict_area[:, :, 1]  # BGR value
    green_channel = cv2.erode(green_channel, np.ones((3, 3), np.int8))
    ret, frame_thresh = cv2.threshold(green_channel, 210, 255, cv2.THRESH_BINARY)
    #cv2.imshow("frame_thresh", cv2.resize(frame_thresh, (image_out_width, image_out_height)))
    
    # Defining all Canny parameters
    t_lower = 150 # Lower threshold value in Hysteresis Thresholding
    t_upper = 250 # Upper threshold value in Hysteresis Thresholding
    aperture_size = 5 # Aperture size, Aperture size of the Sobel filter.
    L2Gradient = True # Boolean, Boolean parameter used for more precision in calculating Edge Gradient.
    edge = cv2.Canny(frame_thresh, t_lower, t_upper,
                 apertureSize = aperture_size, 
                 L2gradient = L2Gradient )
    return edge


def find_mid_h_point(edge_canny_img):
    [width, height] = [edge_canny_img.shape[1], edge_canny_img.shape[0]]
    h_hist = np.sum(edge_canny_img, axis=1)
    mid_h_point = np.argmax(h_hist)
    re_frame = 0
    while (mid_h_point > (height*5/7)) | (mid_h_point < (height*1/3)):
        edge_canny_img[mid_h_point - 10:mid_h_point + 10, :] = 0
        h_hist = np.sum(edge_canny_img, axis=1)
        mid_h_point = np.argmax(h_hist)
        re_frame = re_frame + 1
        if re_frame == 10:
            break
    return mid_h_point


def find_mid_w_point(edge_canny_img, mid_h_point):
    global con_detect_flag
    [width, height] = [edge_canny_img.shape[1], edge_canny_img.shape[0]]
    if (mid_h_point > height * 5 / 7) | (mid_h_point < height * 1 / 3):
        con_detect_flag = False
    else:
        w_hist = np.sum(edge_canny_img[mid_h_point - 5: mid_h_point + 5, :], axis=0) # doc
        mid_w_point = (np.max(w_hist.nonzero()) + np.min(w_hist.nonzero())) // 2
        edge_canny_img[:mid_h_point + 20, :] = 0
        con_detect_flag = True
    return mid_w_point


def draw_lane_lines(edited_edge_img, mid_h_point, mid_w_point, original_predict_img):
    global con_detect_flag
    [width, height] = [edited_edge_img.shape[1], edited_edge_img.shape[0]]

    nonzero_left = edited_edge_img[mid_h_point:, :mid_w_point].nonzero()
    nonzero_right = edited_edge_img[mid_h_point:, mid_w_point:].nonzero()

    nonzero_left_x = nonzero_left[1]
    nonzero_left_y = nonzero_left[0] + mid_h_point

    nonzero_right_x = nonzero_right[1] + mid_w_point
    nonzero_right_y = nonzero_right[0] + mid_h_point

    if len(nonzero_right_y) & len(nonzero_right_x) & len(nonzero_left_y) & len(nonzero_left_x):
        
        center_x_point = (nonzero_left_x[len(nonzero_left_x) - 1] + nonzero_right_x[len(nonzero_right_x) - 1]) // 2
        
        # shift center to closer to left lane
        if center_x_point > 10:
            center_x_point = center_x_point-10

        left_fit_pixel = np.polyfit(nonzero_left_y, nonzero_left_x, 2)
        right_fit_pixel = np.polyfit(nonzero_right_y, nonzero_right_x, 2)

        '''
        left_fit_pixel = np.polyfit(nonzero_left_y, nonzero_left_x, 1)
        right_fit_pixel = np.polyfit(nonzero_right_y, nonzero_right_x, 1)
        '''
        
        # Draw the line
        left_plot_y = np.linspace(nonzero_left_y[0], height - 1, height - nonzero_left_y[0])
        right_plot_y = np.linspace(nonzero_right_y[0], height - 1, height - nonzero_right_y[0])
        
        if len(left_plot_y) == len(right_plot_y):

            left_fit_x = left_fit_pixel[0] * left_plot_y ** 2 + left_fit_pixel[1] * left_plot_y + left_fit_pixel[2]
            right_fit_x = right_fit_pixel[0] * right_plot_y ** 2 + right_fit_pixel[1] * right_plot_y + right_fit_pixel[2]
            '''
            left_fit_x = left_fit_pixel[0] * left_plot_y + left_fit_pixel[1]
            right_fit_x = right_fit_pixel[0] * right_plot_y + right_fit_pixel[1]
            '''

            if (max(right_fit_x) < width) & (min(right_fit_x) > 0):
                # pts_left = np.array([np.transpose(np.vstack([left_fit_x, left_plot_y]))])
                # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, right_plot_y])))])
                # center_fit_x = (left_fit_x + right_fit_x) // 2
                # # pts_center = np.array([np.transpose(np.stack([center_fit_x, left_plot_y]))])
                # pts = np.hstack((pts_left, pts_right))
                
                and_result = np.ones_like(original_predict_img) * 255
                and_result[np.int_(left_plot_y), np.int_(left_fit_x), 1:] = 0
                
                and_result[np.int_(right_plot_y), np.int_(right_fit_x), 1] = 0
                and_result[np.int_(right_plot_y), np.int_(right_fit_x), 0] = 0

                # Draw lane output
                and_result = cv2.erode(and_result, np.ones((3, 3), np.uint8))
                result_frame = cv2.bitwise_and(original_predict_img, and_result)

                left_calc = [left_plot_y, left_fit_x]
                right_calc = [right_plot_y, right_fit_x]
                para_to_calc = [left_calc, right_calc, center_x_point]
            else:
                con_detect_flag = False
    else:
        con_detect_flag = False
    return result_frame, para_to_calc


def find_lines(predict_area, original_predict_img):

    #cv2.imshow("Hinh 1: Predicted_area ", cv2.resize(predict_area, (image_out_width, image_out_height)))

    edge_canny_img = edge_Canny_lanes(predict_area)
    #cv2.imshow("Hinh 2: Default Canny", cv2.resize(edge_canny_img, (image_out_width, image_out_height)))

    mid_h_point = find_mid_h_point(edge_canny_img)
    mid_w_point = find_mid_w_point(edge_canny_img, mid_h_point)
    
    if con_detect_flag:
        result_frame, para_to_calc = draw_lane_lines(edge_canny_img, mid_h_point, mid_w_point, original_predict_img)
        #cv2.imshow("Hinh 4: Output with lane line", cv2.resize(result_frame, (image_out_width, image_out_height)))
    else:
        print("Error")
        result_frame = original_predict_img

    #print (mid_h_point)
    #print (mid_w_point)
    #Edited_Canny_img = edge_canny_img
    #Edited_Canny_img = cv2.circle(Edited_Canny_img, (mid_w_point, mid_h_point), radius=5, color=(255, 255, 255), thickness=5)
    #cv2.imshow("Edited Canny", cv2.resize(Edited_Canny_img, (image_out_width, image_out_height)))
        
    return result_frame, para_to_calc

def convertScale(img, alpha, beta):
    """Add bias and gain to an image with saturation arithmetics. Unlike
    cv2.convertScaleAbs, it does not take an absolute value, which would lead to
    nonsensical results (e.g., a pixel at 44 with alpha = 3 and beta = -210
    becomes 78 with OpenCV, when in fact it should become 0).
    """
    new_img = img * alpha + beta
    new_img[new_img < 0] = 0
    new_img[new_img > 255] = 255
    return new_img
def slope(x1, y1, x2, y2):

    if x1 == x2 :
        if y1 > y2:
            return -99
        else:
            return 99

    else :
        s = (float(y2)-y1)/(x2-x1)
        if s == 0:
            return 0.01
        return s

def avg_lines(lines):

    l_lane = []
    r_lane = []
    l_weight = []
    r_weight = []

    if lines is None:
        return 0, 0
    else:
        for line in lines :
            for x1, y1, x2, y2 in line :
                s = slope(x1, y1, x2, y2)
                c = y1 - s*x1
                length = np.sqrt((y2-y1)**2 + (x2-x1)**2)

                if s < 0:
                    l_lane.append((s, c))
                    l_weight.append((length))

                if s > 0:
                    r_lane.append((s, c))
                    r_weight.append((length))

    if len(l_weight) > 0:
        l_lane = np.dot(l_weight, l_lane)/np.sum(l_weight)
    else:
        None

    if len(r_weight) > 0:
        r_lane = np.dot(r_weight, r_lane)/np.sum(r_weight)
    else:
        None

    return l_lane, r_lane

def get_line(m, c, y1, y2):

    x1 = (int)((y1-c)/m)
    x2 = (int)((y2-c)/m)
    y1 = (int)(y1)
    y2 = (int)(y2)

    return x1, y1, x2, y2

def ROI_mask(image):
    
    height = image.shape[0]
    width = image.shape[1]

        
    # A triangular polygon to segment the lane area and discarded other irrelevant parts in the image
    # Defined by three (x, y) coordinates    
    polygons = np.array([ 
        [(round(width*1/8), height), 
        (round(width*1/3), round(height*3/5)), 
        (round(width*2/3), round(height*3/5)), 
        (round(width*7/8), height)] 
        ]) 
        
    mask = np.zeros_like(image) 
    cv2.fillPoly(mask, polygons, 255)  ## 255 is the mask color
        
    # Bitwise AND between canny image and mask image
    masked_image = cv2.bitwise_and(image, mask)
        
    return masked_image

def hough_lines (img, frame):
    #lines = cv2.HoughLinesP(cropped_image, 1, np.pi/180, 30, np.array([]), min_length, max_gap)
    
    lines = cv2.HoughLinesP(
    img,
    rho=2,              #Distance resolution in pixels
    theta=np.pi/5,  #Angle resolution in radians
    threshold=10,      #Min. number of intersecting points to detect a line  
    lines=np.array([]), #Vector to return start and end points of the lines indicated by [x1, y1, x2, y2] 
    minLineLength=70,   #Line segments shorter than this are rejected
    maxLineGap=300       #Max gap allowed between points on the same line
    )

    #print (lines)
    if lines is None:
        pass
    else:   
        l_lane, r_lane = avg_lines(lines)
        mask = np.zeros_like(frame) 

        if len(l_lane) > 0:
             x1_l, y1_l, x2_l, y2_l = get_line(l_lane[0], l_lane[1], img.shape[0], img.shape[0]*0.7)                    
             cv2.line(mask, (x1_l, y1_l), (x2_l, y2_l), (255, 255, 255), 6, cv2.LINE_AA)
             frame = cv2.addWeighted(frame, 1, mask, 1.0, 0.0)
        else :
            pass


        if len(r_lane) > 0: 
            x1_r, y1_r, x2_r, y2_r = get_line(r_lane[0], r_lane[1], img.shape[0], img.shape[0]*0.7) 
            cv2.line(mask, (x1_r, y1_r), (x2_r, y2_r), (255, 255, 255), 6, cv2.LINE_AA)
            frame = cv2.addWeighted(frame, 1, mask, 1.0, 0.0)
        else:   
            pass

    return frame


def extract_colour(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    sensitivity_white = 115
    lower_white = np.array([0,0,255-sensitivity_white])
    upper_white = np.array([255,255,255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    sensitivity_yellow = 255
    lower_yellow = np.array([0,0,135])
    upper_yellow = np.array([70,sensitivity_yellow,255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.bitwise_or(mask_white, mask_yellow)

    frame = cv2.bitwise_and(img, img, mask = mask)
    return frame

def main(video_filename):
    global con_detect_flag
    cap = cv2.VideoCapture(video_filename) # test using video
    #cap = cv2.VideoCapture(0) # test using camera
    print("=================== Started =====================")
    
    ret, frame = cap.read()
    if ret:
        height = frame.shape[0]
        width = frame.shape[1]   
        polygons = np.array([ 
            [(round(width*1/20), height), 
            (round(width*1/3), round(height*3/5)), 
            (round(width*2/3), round(height*3/5)), 
            (round(width*19/20), height)] 
            ])      
        mask = np.zeros_like(frame) 
        cv2.fillPoly(mask, polygons, color=(1, 1, 1)) 

    convertScale_alpha = 1.2 # Contrast control (1.0-3.0)
    convertScale_beta = -25 # Brightness control (-100 ->100)

    start_frame_time = time.time()
    i = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        
        if ret:
            #yellow_white = extract_colour(frame)
            #cv2.imshow("yellow_white Image", cv2.resize(yellow_white, (image_out_width, image_out_height)))
            #canny = cv2.Canny(yellow_white, 100, 255)
            #cv2.imshow("Canny Image", cv2.resize(canny, (image_out_width, image_out_height)))
            #cropped_image = ROI_mask(canny)
            #cv2.imshow("ROI_mask Image", cv2.resize(cropped_image, (image_out_width, image_out_height)))
            #frame = hough_lines(cropped_image, frame)
            #cv2.imshow("Hough_lines Image", cv2.resize(frame, (image_out_width, image_out_height)))
            
            #frame = convertScale(yellow_white, alpha=convertScale_alpha, beta=convertScale_beta)
            #cv2.imshow("ConvertScale Image", cv2.resize(frame, (image_out_width, image_out_height)))

            #frame = frame * mask
            #cv2.imshow("Masked Image", cv2.resize(frame, (image_out_width, image_out_height)))

            kenerl_size = (5, 5)
            frame = cv2.GaussianBlur(frame, kenerl_size, 0)

            pre_prediction, prediction = predict_lane(frame)
            #cv2.imshow("Area prediction", cv2.resize(pre_prediction, (image_out_width, image_out_height)))
            #cv2.imshow("Default prediction", cv2.resize(prediction, (image_out_width, image_out_height))
            
            try:
                con_detect_flag = True
                output_img, para_to_calc = find_lines(pre_prediction, prediction)
                img_size = [output_img.shape[0], output_img.shape[1]] # [height, width]
                cal_curvature_and_offset(para_to_calc[0], para_to_calc[1], para_to_calc[2], img_size)

            except:
                output_img = prediction

        
            cv2.imshow("Final result", cv2.resize(output_img, (image_out_width, image_out_height)))
            
        else:
            break
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        stop_frame_time = time.time()
        FPS = 1 / (stop_frame_time - start_frame_time)
        print(FPS)
        start_frame_time = stop_frame_time
        i = i + 1
    
    print("=================== Finished =====================")
    cv2.destroyAllWindows()
    cap.release()


if __name__ == '__main__':
    #video_name = "video/20220423/20220423_161502.mp4"
    #video_name = "video/20220423/20220423_161605.mp4"
    #video_name = "video/20220423/20220423_161733.mp4"
    #video_name = "video/20220423/20220423_161853.mp4"
    #video_name = "video/20220423/4.mp4"
    
    #video_name = "video/20220424/5.mp4"
    video_name = "video/20220424/1_cropped.mp4"
    
    #video_name = "video/VAS/vid_1.mp4"
    #video_name = "video/hcm.mp4"
    main(video_name)
