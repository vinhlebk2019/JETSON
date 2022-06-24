import numpy as np
from tensorflow import keras
from tensorflow.keras.models import load_model
import tensorflow as tf
import cv2
import time
import rospy
from aev_pkg.msg import lane_detection_msg
from aev_pkg.msg import gui_msg

# Class to average lanes with
class Lanes:
    def __init__(self):
        self.recent_fit = []
        self.avg_fit = []

class out_paras:
    def __init__(self):
        self.model_available = []
        self.radius_model = []
        self.offset_model = []

        self.polyfit_available = []
        self.radius_polyfit = []
        self.offset_polyfit = []

        self.houghline_available = []
        self.offset_houghline = []

        self.msg = ""
        self.error_msg = ""
        self.pre_radius = None
        self.pre_offset = []

########## ROS Parameters ##########
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'ooo' node so that multiple listeners can
# run simultaneously.
rospy.init_node('LaneDetect_Node', anonymous=True)
pub_lane_data = rospy.Publisher('LaneDetc_Data', lane_detection_msg, queue_size=10)
lane_detection_data = lane_detection_msg()

########## Define ##########
invalid_value = 10000
# Lane parameter
lanes = Lanes()
# Load model
model = load_model('full_CNN_model.h5')
# all output data
output_data = out_paras()
# turn value
turn_none = 0
turn_left = 1
turn_right = 2

########## Variables ##########
# steeringLeftRight
gui_steeringLeftRight = 0
# gui_previewCurvature
gui_previewCurvature = 0.0
# continue_detect_flag
con_detect_flag = True
# finalCenterOffset
finalCenterOffset = 0
# ros publish msg counter
ros_msg_counter = 0
# turn history
turn_cnt = 0
turn_direction = 0
direction_history = 0
# center value
x_center1 = 0
x_center2 = 0
x_center_diff = 0

########## Config Parameters ##########
# Tinh ra don vi meter tren pixel
height_to_meter = 30
width_to_meter = 3.5
# kich thuoc anh ngo ra
image_out_width = 800
image_out_height = 600
#
cfg_erode_size = 10
cfg_dilate_length = 20
# Prediction avarage
average = 7
# draw houghLin or not
draw_houghLine = 0
# shift_left_center_offset
shift_left_center_offset = 0
# houghLine_width_deviation
houghLine_width_deviation = 80
# polyfit order
polyfit_order = 1
# direction_change_frame_num_thres
direction_change_frame_num_thres = 20
# polyfit order
direction_change_center_diff_thres = 60
# Config accepted line horizon differential
line_horizon_thres = 300
# Config algorithm 
using_houghLine_flag = 1 # 0 or 1
using_polyfit_flag = 1 # 0 or 1
using_Model_flag = 0 # 0 or 1
# Config ROI (mask) 
cfg_mask_width_factor = 3 
cfg_mask_height_factor = 2.5
cfg_mask_height_derating_factor = 0.5
cfg_mask_center_shift_factor = 0.7
cfg_mask_devidied = 5
cfg_mask_offset_factor = 1.8

################################################################################
def callback_gui_msg(data):
    global gui_steeringLeftRight
    global gui_previewCurvature
    rospy.loginfo("Received Gui data %d" % data.msg_counter)
    #print("Comment: Received speedSetpoint: {}".format(data.speedSetpoint))
    gui_steeringLeftRight = data.steeringLeftRight
    gui_previewCurvature = data.previewCurvature
rospy.Subscriber("GUI_Data", gui_msg, callback_gui_msg)

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
    output_data.radius_model = (left_curveR + right_curveR) // 2

    # chia 10^16 de de quan sat, km dung thi chia 1000, m thi khong can chia
    output_data.radius_model = output_data.radius_model / 10000000

    # Calculate Center offset
    output_data.offset_model = (last_pt_center - (width // 2)) * xm_per_pix


def edge_Canny_lanes(predict_area):
    green_channel = predict_area[:, :, 1]  # BGR value
    green_channel = cv2.erode(green_channel, np.ones((3, 3), np.int8))
    ret, frame_thresh = cv2.threshold(green_channel, 150, 255, cv2.THRESH_BINARY)
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
        edge_canny_img[mid_h_point - 3 : mid_h_point + 3, :] = 0
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
        w_hist = np.sum(edge_canny_img[mid_h_point - 3: mid_h_point + 3, :], axis=0) # doc
        mid_w_point = (np.max(w_hist.nonzero()) + np.min(w_hist.nonzero())) // 2
        edge_canny_img[:mid_h_point + 3, :] = 0
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
        
        '''
        center_x_point = (nonzero_left_x[len(nonzero_left_x) - 1] + nonzero_right_x[len(nonzero_right_x) - 1]) // 2
        # shift center to closer to left lane
        if center_x_point > 10:
            center_x_point = center_x_point - shift_left_center_offset
        '''

        left_fit_pixel = np.polyfit(nonzero_left_y, nonzero_left_x, polyfit_order)
        right_fit_pixel = np.polyfit(nonzero_right_y, nonzero_right_x, polyfit_order)

        '''
        left_fit_pixel = np.polyfit(nonzero_left_y, nonzero_left_x, 1)
        right_fit_pixel = np.polyfit(nonzero_right_y, nonzero_right_x, 1)
        '''
        
        # Draw the line
        left_plot_y = np.linspace(nonzero_left_y[0], height - 1, height - nonzero_left_y[0])
        right_plot_y = np.linspace(nonzero_right_y[0], height - 1, height - nonzero_right_y[0])
        
        if len(left_plot_y) == len(right_plot_y):

            if (polyfit_order == 2):
                left_fit_x = left_fit_pixel[0] * left_plot_y ** 2 + left_fit_pixel[1] * left_plot_y + left_fit_pixel[2]
                right_fit_x = right_fit_pixel[0] * right_plot_y ** 2 + right_fit_pixel[1] * right_plot_y + right_fit_pixel[2]
            else:
                left_fit_x = left_fit_pixel[0] * left_plot_y + left_fit_pixel[1]
                right_fit_x = right_fit_pixel[0] * right_plot_y + right_fit_pixel[1]
            
            '''
            original_predict_img = cv2.circle(
                        img = original_predict_img, 
                        center = (round(left_fit_x[len(left_plot_y) - 1]), height), #width, height
                        radius = 5, 
                        color = (255,0,0), 
                        thickness = 2)
            original_predict_img = cv2.circle(
                        img = original_predict_img, 
                        center = (round(right_fit_x[len(right_plot_y) -1]), height), #width, height
                        radius = 5, 
                        color = (0,0,255), 
                        thickness = 2)
            '''

            center_x_point = (round(left_fit_x[len(left_plot_y) - 1]) + round(right_fit_x[len(right_plot_y) -1])) // 2
            # shift center to closer to left lane
            if center_x_point > 10:
                center_x_point = center_x_point + shift_left_center_offset
            

            if (max(right_fit_x) < width) & (min(right_fit_x) > 0):
                # pts_left = np.array([np.transpose(np.vstack([left_fit_x, left_plot_y]))])
                # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, right_plot_y])))])
                # center_fit_x = (left_fit_x + right_fit_x) // 2
                # # pts_center = np.array([np.transpose(np.stack([center_fit_x, left_plot_y]))])
                # pts = np.hstack((pts_left, pts_right))
                
                and_result = np.ones_like(original_predict_img) * 255

                # Left line with Blue color
                and_result[np.int_(left_plot_y), np.int_(left_fit_x), 1:] = 0
                
                # Blue line with Red color
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
                if abs(y1 - y2) > houghLine_width_deviation:
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

    return len(l_weight), len(r_weight), l_lane, r_lane

def get_line(m, c, y1, y2):
    x1 = (int)((y1-c)/m)
    x2 = (int)((y2-c)/m)
    y1 = (int)(y1)
    y2 = (int)(y2)
    return x1, y1, x2, y2

def hough_lines (img, frame):
    lines = cv2.HoughLinesP(
    img,
    rho=4,              #Distance resolution in pixels
    theta=np.pi/90,      #Angle resolution in radians
    threshold=4,       #Min. number of intersecting points to detect a line  
    lines=np.array([]), #Vector to return start and end points of the lines indicated by [x1, y1, x2, y2] 
    minLineLength=100,   #Line segments shorter than this are rejected
    maxLineGap=400       #Max gap allowed between points on the same line
    )

    center1 = invalid_value
    center2 = invalid_value

    if lines is None:
        pass
    else:   
        if (draw_houghLine == 1):
            for i in range(0, len(lines)):
                l = lines[i][0]
                if abs(l[1] - l[3]) > houghLine_width_deviation:
                    cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (255,255,255), 3, cv2.LINE_AA)

        weight_1, weight_2, l_lane, r_lane = avg_lines(lines)
        
        # left lane
        if (len(l_lane) > 0) and (len(r_lane) > 0):

            if weight_1 > weight_2:
                x1_l, y1_l, x2_l, y2_l = get_line(l_lane[0], l_lane[1], img.shape[0], img.shape[0]*0.7)
                if (abs(x1_l - x2_l) < line_horizon_thres):             
                    cv2.line(frame, (x1_l, y1_l), (x2_l, y2_l), (255, 255, 255), 10, cv2.LINE_AA)

            else:
                x1_l, y1_l, x2_l, y2_l = get_line(r_lane[0], r_lane[1], img.shape[0], img.shape[0]*0.7)
                if (abs(x1_l - x2_l) < line_horizon_thres):                  
                    cv2.line(frame, (x1_l, y1_l), (x2_l, y2_l), (255, 255, 255), 10, cv2.LINE_AA)

            center1 = x1_l
            center2 = x2_l

        elif (len(l_lane) > 0):
            x1_l, y1_l, x2_l, y2_l = get_line(l_lane[0], l_lane[1], img.shape[0], img.shape[0]*0.7)
            if (abs(x1_l - x2_l) < line_horizon_thres):                     
                cv2.line(frame, (x1_l, y1_l), (x2_l, y2_l), (255, 255, 255), 10, cv2.LINE_AA)
            center1 = x1_l
            center2 = x2_l

        elif (len(r_lane) > 0):
            x1_l, y1_l, x2_l, y2_l = get_line(r_lane[0], r_lane[1], img.shape[0], img.shape[0]*0.7)
            if (abs(x1_l - x2_l) < line_horizon_thres):                
                cv2.line(frame, (x1_l, y1_l), (x2_l, y2_l), (255, 255, 255), 10, cv2.LINE_AA)
            center1 = x1_l
            center2 = x2_l
            
        else :
            center1 = frame.shape[1] // 2 # width // 2
            center2 = frame.shape[1] // 2 # width // 2

        #print(float(x2_l - x1_l))

    return frame, center1, center2

def draw_direction(img, direction, pre_curvature, height, width):

    if (direction == 1):
        color_left = 255
        color_right = 0
        color_straight = 0
    elif (direction == 2):
        color_left = 0
        color_right = 255
        color_straight = 0
    else:
        color_left = 0
        color_right = 0
        color_straight = 255

    # Left
    result_frame = cv2.arrowedLine(
                    img = img, 
                    pt1 = (100, height - 100), 
                    pt2 = (50, height - 100),
                    color = (0,color_left,0), 
                    thickness = 10,
                    tipLength = 0.4)

    # Right
    result_frame = cv2.arrowedLine(
                    img = result_frame, 
                    pt1 = (100, height - 100), 
                    pt2 = (150, height - 100),
                    color = (0,color_right,0), 
                    thickness = 10,
                    tipLength = 0.4)

    # Straight
    result_frame = cv2.arrowedLine(
                    img = result_frame, 
                    pt1 = (100, height - 100), 
                    pt2 = (100, height - 150),
                    color = (0,color_straight,0), 
                    thickness = 10,
                    tipLength = 0.4)

    # msg = "Preview Curvature {:.2f} m".format(pre_curvature)
    # cv2.putText(result_frame, 
    #                 msg, 
    #                 org=(0, height - 50), 
    #                 fontFace=cv2.FONT_HERSHEY_SIMPLEX,              
    #                 fontScale=1, 
    #                 color=(0, 255, 0), 
    #                 thickness=2)

    return result_frame

def extract_colour(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    sensitivity_vmin = 160 #140 #155 # cang to cang den con duong
    sensitivity_hmax = 40 #40 #51 # cang nho cang den bau troi
    lower = np.array([0,0,sensitivity_vmin])
    upper = np.array([sensitivity_hmax,255,255])
    mask = cv2.inRange(hsv, lower, upper)

    frame = cv2.bitwise_and(img, img, mask = mask)
    return frame

def polyfit_lane(img_input_bw, img_rgb , height, width, mid_h, mid_w):

    line_seg_x_low = width // 2
    line_seg_x_high = width // 2

    #mid_h_point = round(height * 1/2)
    #mid_w_point = round(width * 1/2)

    nonzero_left = img_input_bw[mid_h:, :mid_w].nonzero()

    nonzero_left_x = nonzero_left[1]
    nonzero_left_y = nonzero_left[0] + mid_h
    try:
        left_fit_pixel = np.polyfit(nonzero_left_y, nonzero_left_x, polyfit_order)

        left_plot_y = np.linspace(nonzero_left_y[0], height - 1, height - nonzero_left_y[0])

        if (polyfit_order == 2):
            left_fit_x = left_fit_pixel[0] * left_plot_y ** 2 + left_fit_pixel[1] * left_plot_y + left_fit_pixel[2]
        else:
            left_fit_x = left_fit_pixel[0] * left_plot_y + left_fit_pixel[1]
        
        and_result = np.ones_like(img_rgb) * 255
        and_result[np.int_(left_plot_y), np.int_(left_fit_x), 1:] = 0

        # Draw lane output
        and_result = cv2.erode(and_result, np.ones((10, 10), np.uint8))
        result_frame = cv2.bitwise_and(img_rgb, and_result)

        line_seg_x_low = round(left_fit_x[len(left_plot_y) - 1])
        line_seg_x_high = round(left_fit_x[0])
    except:
        result_frame = img_rgb
        pass

    return result_frame, line_seg_x_low, line_seg_x_high

def get_mask(ref_frame, mask_width, mask_height, devidied, x_point, y_point, offset):
    div = devidied*2
    polygons = np.array([ 
        [(round(x_point - (mask_width//2)), round(y_point)), 
        (round(x_point - (mask_width//div) + offset), round(y_point - mask_height)), 
        (round(x_point + (mask_width//div) + offset), round(y_point - mask_height)), 
        (round(x_point + (mask_width//2)), round(y_point))] 
        ])     
    mask_all = np.zeros_like(ref_frame) 
    cv2.fillPoly(mask_all, polygons, color=(1, 1, 1))

    mask = mask_all[:,:]
    return mask

arr_moving_average = [0, 0, 0, 0, 0, 0]
def moving_average(new):
    global arr_moving_average

    # Add new element to array
    i = 0
    while i < len(arr_moving_average) - 1:
        arr_moving_average[i] = arr_moving_average[i+1]
        i += 1
    arr_moving_average[len(arr_moving_average) - 1] = new
    
    # Store cumulative sums of array in cum_sum array
    cum_sum = np.cumsum(arr_moving_average)

    i = 1
    # Loop through the array elements
    while i <= len(arr_moving_average):
    
        # Calculate the cumulative average by dividing
        # cumulative sum by number of elements till 
        # that position
        window_average = round(cum_sum[i-1] / i, 2)
        
        # Shift window to right by one position
        i += 1
    
    return(window_average)

def main(video_filename):
    global con_detect_flag
    global ros_msg_counter
    global finalCenterOffset
    global x_center1
    global x_center2
    global x_center_diff
    global turn_cnt
    global turn_direction
    global direction_history

    cap = cv2.VideoCapture(video_filename)

    print("=================== Started =====================")

    while (1):
        ret, frame = cap.read()
        if ret:
            height = frame.shape[0]
            width = frame.shape[1] 
            break
        else:
            height = 720
            width = 1080 

    print("=================== Start Capture =====================")
    start_frame_time = time.time()
    start_time = time.time()

    x_center1 = width//2
    x_center2 = width//2
    
    while (cap.isOpened()):

        ret, frame = cap.read()

        if ret:
            #cv2.imshow("Org Image", cv2.resize(frame, (image_out_width, image_out_height)))

            yellow_white = extract_colour(frame)

            se=cv2.getStructuringElement(cv2.MORPH_RECT , (5,5))
            yellow_white=cv2.morphologyEx(yellow_white, cv2.MORPH_DILATE, se)
            #cv2.imshow("yellow_white Image", cv2.resize(yellow_white, (image_out_width, image_out_height)))
            
            #binary_img = cv2.Canny(yellow_white, 100, 255)
            binary_img = cv2.cvtColor(yellow_white, cv2.COLOR_BGR2GRAY)

            a, binary_img = cv2.threshold(binary_img, 120, 255, cv2.THRESH_BINARY)

            # mask_1 = get_mask(ref_frame=binary_img, 
            #                     mask_width=(width // (cfg_mask_width_factor*1.0)), 
            #                     mask_height=((height // (cfg_mask_height_factor*1.0)) - cfg_mask_height_derating_factor*abs(x_center_diff)), 
            #                     devidied=cfg_mask_devidied, 
            #                     x_point=(x_center2 + x_center1)/2 - cfg_mask_center_shift_factor*x_center_diff, 
            #                     y_point=height, 
            #                     offset=cfg_mask_offset_factor*x_center_diff)

            # # mask_1_invert = np.ones_like(binary_img) 
            # # mask_1_invert = mask_1_invert - mask_1
            # # binary_img_invert = binary_img * mask_1_invert

            # kernel = np.ones((cfg_erode_size,cfg_erode_size), np.uint8)
            # binary_img_eroded = cv2.erode(binary_img, kernel, iterations=1)

            # binary_img_masked = binary_img * mask_1
            # kernel = np.ones((cfg_dilate_length,3), np.uint8)
            # binary_img_dilated = cv2.dilate(binary_img_masked, kernel, iterations=1)
            # binary_img = cv2.bitwise_or(binary_img_eroded, binary_img_dilated)

            cv2.imshow("Binary Image", cv2.resize(binary_img, (image_out_width, image_out_height)))
            
            gui_steeringLeftRight = 0
            
            ####################################################################################################

            x_center_diff = x_center2 - x_center1
            if (x_center_diff > 100):
                x_center_diff = 100
            elif (x_center_diff < -100):
                x_center_diff = -100

            if (using_houghLine_flag == 1):

                mask_all = get_mask(ref_frame=binary_img, 
                                    mask_width=(width // cfg_mask_width_factor), 
                                    mask_height=((height // cfg_mask_height_factor) - cfg_mask_height_derating_factor*abs(x_center_diff)), 
                                    devidied=cfg_mask_devidied, 
                                    x_point=(x_center2 + x_center1)/2 - cfg_mask_center_shift_factor*x_center_diff, 
                                    y_point=height, 
                                    offset=cfg_mask_offset_factor*x_center_diff)
                cropped_image = binary_img * mask_all

                frame, x_c1, x_c2 = hough_lines(cropped_image, frame)
                
                if (x_c1 != invalid_value) and (x_c2 != invalid_value):

                    if (x_c1 < 0) or (x_c1 > width) or (x_c2 < 0) or (x_c2 > width):
                        x_c1 = width//2
                        x_c2 = width//2

                    if (abs(x_c1 - x_center1) > 100):
                        x_c1 = x_center1
                        x_c2 = x_center2

                    frame = cv2.circle(
                        img = frame, 
                        center = (round(x_c1), height), #width, height
                        radius = 5, 
                        color = (0,0,255),
                        thickness = 3)
                    #frame[:,:,1] = frame[:,:,1] * mask_all[:,:,1]

                    output_data.offset_houghline = (round(x_c1) - (width // 2)) * (width_to_meter / width)
                    output_data.houghline_available = 1
                else :
                    x_c1 = width//2
                    x_c2 = width//2
                    output_data.offset_houghline = 0
                    output_data.houghline_available = 0
            else:
                x_c1 = width//2
                x_c2 = width//2
                output_data.offset_houghline = 0
                output_data.houghline_available = 0

            ####################################################################################################
            if (using_polyfit_flag == 1):
                '''
                mask_width_init = width // 3
                mask_height_init = height // 9

                mask1 = get_mask(binary_img, mask_width_init, mask_height_init, 1.5, x_center1, height)
                cropped_image = binary_img * mask1
                frame1, x_low, x_high = polyfit_lane(cropped_image, frame, height, width, height // 2, x_center1)

                mask2 = get_mask(binary_img, mask_width_init / 2, mask_height_init, 1.5, x_high, height - (mask_height_init + 1))
                cropped_image = binary_img * mask2
                frame2, x_low, x_high = polyfit_lane(cropped_image, frame, height, width, height // 2, x_high)

                mask3 = get_mask(binary_img, mask_width_init / 4, mask_height_init, 2, x_high, height - (2*mask_height_init + 2))
                mask = mask1 + mask2 + mask3
                '''
                mask = get_mask(ref_frame=binary_img, 
                                mask_width=(width // cfg_mask_width_factor), 
                                mask_height=((height // cfg_mask_height_factor) - cfg_mask_height_derating_factor*abs(x_center_diff)), 
                                devidied=cfg_mask_devidied, 
                                x_point=(x_center2 + x_center1)/2 - cfg_mask_center_shift_factor*x_center_diff, 
                                y_point=height, 
                                offset=cfg_mask_offset_factor*x_center_diff)

                cropped_image1 = binary_img * mask

                # Defining all Canny parameters
                t_lower = 150 # Lower threshold value in Hysteresis Thresholding
                t_upper = 250 # Upper threshold value in Hysteresis Thresholding
                aperture_size = 5 # Aperture size, Aperture size of the Sobel filter.
                L2Gradient = True # Boolean, Boolean parameter used for more precision in calculating Edge Gradient.
                #canny = cv2.Canny(cropped_image, t_lower, t_upper, apertureSize = aperture_size, L2gradient = L2Gradient )

                frame, x_low, x_high = polyfit_lane(cropped_image1, frame, height, width, height // 2, width // 2)

                frame = cv2.circle(
                        img = frame, 
                        center = (round(x_low), height), #width, height
                        radius = 5, 
                        color = (0,0,255), 
                        thickness = 3)
                
                frame[:,:,1] = frame[:,:,1] * mask

                #cv2.imshow("Image Polyfit", cv2.resize(frame, (image_out_width, image_out_height)))

                output_data.polyfit_available = 1
                output_data.offset_polyfit = (round(x_low) - (width // 2)) * (width_to_meter / width)
            else:
                output_data.polyfit_available = 0
                output_data.offset_polyfit = 0

            # frame11 = frame
            # frame11[:,:,1] = frame11[:,:,1] * mask_all
            # cv2.imshow("Image HoughLine", cv2.resize(frame11, (image_out_width, image_out_height)))

            ####################################################################################################
            
            if (abs(x_c2 - x_c1) > line_horizon_thres):
                pass
            elif (x_c2 - x_c1) > direction_change_center_diff_thres:
                turn_direction = turn_right
            elif (x_c2 - x_c1) < -direction_change_center_diff_thres:
                turn_direction = turn_left
            else:
                turn_direction = turn_none

            if (direction_history != turn_direction):
                turn_cnt = turn_cnt + 1
                if turn_cnt > direction_change_frame_num_thres:
                    direction_history = turn_direction
            else:
                turn_cnt = 0
                x_center1 = x_c1
                x_center2 = x_c2

            if (x_center1 < 400) or (x_center1 > (width - 400)):
                x_center1 = width // 2
                x_center2 = width // 2
            
            if (direction_history == turn_left):
                if (x_center2 > x_center1):
                    x_center2 = x_center1 - 50
            elif (direction_history == turn_right):
                if (x_center2 < x_center1):
                    x_center2 = x_center1 + 50
            #print (x_center1)

            ####################################################################################################

            if (output_data.polyfit_available == 1) and (output_data.houghline_available == 1):
                finalCenterOffset = (output_data.offset_polyfit + output_data.offset_houghline) / 2
            elif (output_data.polyfit_available == 1):
                finalCenterOffset = output_data.offset_polyfit
            elif (output_data.houghline_available == 1):
                finalCenterOffset = output_data.offset_houghline
            else:
                pass
            
            if (x_center2 - x_center1) == 0:
                radius = 10000
            else:
                radius = -1 / (x_center2 - x_center1) * 1000
            radius = moving_average(radius)
            finalCurvature = 1 / radius

            ros_msg_counter = ros_msg_counter + 1
            lane_detection_data.msg_counter = ros_msg_counter
            lane_detection_data.centerOffset = -finalCenterOffset
            lane_detection_data.curvature = finalCurvature * 0.1
            pub_lane_data.publish(lane_detection_data)
            
            output_img = frame
            #output_img = draw_direction(output_img, gui_steeringLeftRight, gui_previewCurvature, height, width)
            output_img = draw_direction(output_img, direction_history, gui_previewCurvature, height, width)
            
            centerOffset_msg = "Offset = {:.2f} m".format(-finalCenterOffset)
            curvature_msg = "Curvature Radius = {:.2f} m".format(radius)
            output_data.msg = centerOffset_msg + curvature_msg
            cv2.putText(output_img, 
                        centerOffset_msg, 
                        org=(0, 50), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,              
                        fontScale=1, 
                        color=(0, 255, 0), 
                        thickness=2)
            cv2.putText(output_img, 
                        curvature_msg, 
                        org=(0, 100), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,              
                        fontScale=1, 
                        color=(0, 255, 0), 
                        thickness=2)

            output_img = cv2.arrowedLine(
                        img = output_img, 
                        pt1 = (width // 2, height), 
                        pt2 = (width // 2, height - 20),
                        color = (0,0,255), 
                        thickness = 2,
                        tipLength = 0.3)

            cv2.imshow("Final result", cv2.resize(output_img, (image_out_width, image_out_height)))

            stop_frame_time = time.time()
            FPS = 1 / (stop_frame_time - start_frame_time)
            print(FPS)
            start_frame_time = stop_frame_time
            #time.sleep(0.05)
            
        else:
            pass
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()

    stop_time = time.time()
    print("Total")
    print(stop_time - start_time)

    ros_msg_counter = ros_msg_counter + 1
    lane_detection_data.msg_counter = ros_msg_counter
    lane_detection_data.centerOffset = 0
    lane_detection_data.curvature = 0
    pub_lane_data.publish(lane_detection_data)
    print("=================== Finished =====================")
    cv2.destroyAllWindows()


if __name__ == '__main__':

    lane_detection_data.msg_counter = ros_msg_counter
    lane_detection_data.centerOffset = 0.0
    lane_detection_data.curvature = 0.0

    #video_name = "/home/ubuntu/aev/videos/20220424/1_cropped.mp4"
    
    #video_name = "/home/ubuntu/aev/videos/20220428/1.mp4"

    #video_name = "/home/ubuntu/aev/videos/20220507/3.mp4"

    video_name = "/home/ubuntu/aev/videos/20220603/1.mp4"

    #video_name = 0

    main(video_name)
