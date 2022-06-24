import numpy as np
from tensorflow import keras
from tensorflow.keras.models import load_model
import tensorflow as tf
import cv2
import time
import rospy
from aev_pkg.msg import lane_detection_msg
from aev_pkg.msg import gui_msg
import pyrealsense2 as rs #pip3 install pyrealsense2

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

########## Parameters ##########
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'ooo' node so that multiple listeners can
# run simultaneously.
rospy.init_node('LaneDetect_Node', anonymous=True)
pub_lane_data = rospy.Publisher('LaneDetc_Data', lane_detection_msg, queue_size=10)
lane_detection_data = lane_detection_msg()

########## Realsense Camera ##########
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

########## Parameters ##########
invalid_value = 10000
# steeringLeftRight
gui_steeringLeftRight = 0
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
width_to_meter = 3.0
# kich thuoc anh ngo ra
image_out_width = 800
image_out_height = 500
# Prediction avarage
average = 7
# ros publish msg counter
ros_msg_counter = 0
# draw houghLin or not
draw_houghLine = 0
# shift_left_center_offset
shift_left_center_offset = 0
# houghLine_width_deviation
houghLine_width_deviation = 60
# polyfit order
polyfit_order = 2
# Config algorithm 
using_houghLine_flag = 1 # 0 or 1
using_polyfit_flag = 0 # 0 or 1
using_Model_flag = 1 # 0 or 1
#using_roi = "straight" # left - right - straight

def callback_gui_msg(data):
    global gui_steeringLeftRight
    rospy.loginfo("Received Gui data %d" % data.msg_counter)
    #print("Comment: Received speedSetpoint: {}".format(data.speedSetpoint))
    gui_steeringLeftRight = data.steeringLeftRight
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

    return l_lane, r_lane

def get_line(m, c, y1, y2):
    x1 = (int)((y1-c)/m)
    x2 = (int)((y2-c)/m)
    y1 = (int)(y1)
    y2 = (int)(y2)
    return x1, y1, x2, y2


def hough_lines_left (img, frame):
    lines = cv2.HoughLinesP(
    img,
    rho=4,              #Distance resolution in pixels
    theta=np.pi/90,      #Angle resolution in radians
    threshold=4,       #Min. number of intersecting points to detect a line  
    lines=np.array([]), #Vector to return start and end points of the lines indicated by [x1, y1, x2, y2] 
    minLineLength=100,   #Line segments shorter than this are rejected
    maxLineGap=500       #Max gap allowed between points on the same line
    )

    x2_l = invalid_value

    if lines is None:
        pass
    else:   
        if (draw_houghLine == 1):
            for i in range(0, len(lines)):
                l = lines[i][0]
                if abs(l[1] - l[3]) > houghLine_width_deviation:
                    cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (255,255,255), 3, cv2.LINE_AA)

        l_lane, r_lane = avg_lines(lines)
        # left lane
        if len(l_lane) > 0:
             x1_l, y1_l, x2_l, y2_l = get_line(l_lane[0], l_lane[1], img.shape[0], img.shape[0]*0.8)                    
             cv2.line(frame, (x1_l, y1_l), (x2_l, y2_l), (255, 255, 255), 10, cv2.LINE_AA)
        else :
            pass
    
    return frame, x2_l

def hough_lines_right (img, frame):
    lines = cv2.HoughLinesP(
    img,
    rho=7,              #Distance resolution in pixels
    theta=np.pi/45,      #Angle resolution in radians
    threshold=4,       #Min. number of intersecting points to detect a line  
    lines=np.array([]), #Vector to return start and end points of the lines indicated by [x1, y1, x2, y2] 
    minLineLength=150,   #Line segments shorter than this are rejected
    maxLineGap=500       #Max gap allowed between points on the same line
    )

    x2_r = invalid_value

    if lines is None:
        pass
    else:   
        if (draw_houghLine == 1):
            for i in range(0, len(lines)):
                l = lines[i][0]
                if abs(l[1] - l[3]) > houghLine_width_deviation:
                    cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (255,255,255), 3, cv2.LINE_AA)
        
        
        l_lane, r_lane = avg_lines(lines)
        # right lane
        if len(r_lane) > 0: 
            x1_r, y1_r, x2_r, y2_r = get_line(r_lane[0], r_lane[1], img.shape[0], img.shape[0]*0.8) 
            # (mask, lower_point, higher_point, color, thickness, line_type)
            cv2.line(frame, (x1_r, y1_r), (x2_r, y2_r), (255, 255, 255), 10, cv2.LINE_AA)
        else:   
            pass
        
    return frame, x2_r

def extract_colour(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    '''
    sensitivity_white = 255
    lower_white = np.array([0,0,255-sensitivity_white])
    upper_white = np.array([255,255,255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    sensitivity_yellow = 255
    lower_yellow = np.array([0,0,135])
    upper_yellow = np.array([70,sensitivity_yellow,255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    '''
    sensitivity_vmin = 155 #140 #155 # cang to cang den con duong
    sensitivity_hmax = 45 #40 #51 # cang nho cang den bau troi
    lower = np.array([0,0,sensitivity_vmin])
    upper = np.array([sensitivity_hmax,255,255])
    mask = cv2.inRange(hsv, lower, upper)

    #mask = cv2.bitwise_or(mask_white, mask_yellow)

    frame = cv2.bitwise_and(img, img, mask = mask)
    return frame

def polyfit_lane(img_input_bw, img_rgb , height, width):

    #mask11 = np.ones_like(frame)
    #mask11[:,:,0] = img_input
    #mask11[:,:,1] = img_input
    #mask11[:,:,2] = img_input
    #frame = cv2.addWeighted(frame, 1.0, mask11, 1.0, 0.0)
    
    mid_h_point = height //2
    mid_w_point = width //2

    nonzero_left = img_input_bw[mid_h_point:, :mid_w_point].nonzero()
    nonzero_right = img_input_bw[mid_h_point:, mid_w_point:].nonzero()

    nonzero_left_x = nonzero_left[1]
    nonzero_left_y = nonzero_left[0] + mid_h_point

    nonzero_right_x = nonzero_right[1] + mid_w_point
    nonzero_right_y = nonzero_right[0] + mid_h_point

    left_fit_pixel = np.polyfit(nonzero_left_y, nonzero_left_x, 2)
    right_fit_pixel = np.polyfit(nonzero_right_y, nonzero_right_x, 2)

    left_plot_y = np.linspace(nonzero_left_y[0], height - 1, height - nonzero_left_y[0])
    right_plot_y = np.linspace(nonzero_right_y[0], height - 1, height - nonzero_right_y[0])

    left_fit_x = left_fit_pixel[0] * left_plot_y ** 2 + left_fit_pixel[1] * left_plot_y + left_fit_pixel[2]
    right_fit_x = right_fit_pixel[0] * right_plot_y ** 2 + right_fit_pixel[1] * right_plot_y + right_fit_pixel[2]

    and_result = np.ones_like(img_rgb) * 255
    and_result[np.int_(left_plot_y), np.int_(left_fit_x), 1:] = 0
    and_result[np.int_(right_plot_y), np.int_(right_fit_x), 1] = 0
    and_result[np.int_(right_plot_y), np.int_(right_fit_x), 0] = 0

    # Draw lane output
    and_result = cv2.erode(and_result, np.ones((5, 5), np.uint8))
    result_frame = cv2.bitwise_and(img_rgb, and_result)

    return result_frame


def main():
    global con_detect_flag
    global ros_msg_counter

    convertScale_alpha = 1.6 # Contrast control (1.0-3.0)
    convertScale_beta = 0 # Brightness control (-100 ->100)
    erode_kernel = np.ones((5,10), np.uint8)

    print("=================== Started =====================")

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("No camera found!!!")
        exit(0)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
    #config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 8)
    
    # Start streaming
    pipeline.start(config)

    while True:
        realsense_frames = pipeline.wait_for_frames()
        color_frame = realsense_frames.get_color_frame()
        if color_frame:
            frame = np.asanyarray(color_frame.get_data())
            height = frame.shape[0]
            width = frame.shape[1] 
            break
        else:
            pass

    print("=================== Confog ROI =====================")
    polygons_straight_left = np.array([ 
        [(round(width*0/10), round(height)), 
        (round(width*4/10), round(height*6/10)), 
        (round(width*5/10), round(height*6/10)), 
        (round(width*5/10), round(height))] 
        ])      
    mask_straight_left = np.zeros_like(frame)
    cv2.fillPoly(mask_straight_left, polygons_straight_left, color=(1, 1, 1)) 

    polygons_straight_right = np.array([ 
        [(round(width*5/10), round(height)), 
        (round(width*5/10), round(height*6/10)), 
        (round(width*6/10), round(height*6/10)), 
        (round(width*10/10), round(height))] 
        ])      
    mask_straight_right = np.zeros_like(frame) 
    cv2.fillPoly(mask_straight_right, polygons_straight_right, color=(1, 1, 1))

    mask_all_straight = cv2.bitwise_or(mask_straight_left, mask_straight_right)

    ####

    polygons_curvarture_left_1 = np.array([ 
        [(round(width*1/10), round(height)), 
        (round(width*2/10), round(height*6/10)), 
        (round(width*4/10), round(height*6/10)), 
        (round(width*5/10), round(height))] 
        ])      
    mask_curvarture_left_1 = np.zeros_like(frame)
    cv2.fillPoly(mask_curvarture_left_1, polygons_curvarture_left_1, color=(1, 1, 1)) 

    polygons_curvarture_left_2 = np.array([ 
        [(round(width*5/10), round(height)), 
        (round(width*4/10), round(height*6/10)), 
        (round(width*6/10), round(height*6/10)), 
        (round(width*9/10), round(height))] 
        ])    
    mask_curvarture_left_2 = np.zeros_like(frame)
    cv2.fillPoly(mask_curvarture_left_2, polygons_curvarture_left_2, color=(1, 1, 1)) 

    mask_all_curvarture_left = cv2.bitwise_or(mask_curvarture_left_1, mask_curvarture_left_2)

    ####

    polygons_curvarture_right_1 = np.array([ 
        [(round(width*0/10), round(height)), 
        (round(width*4/10), round(height*6/10)), 
        (round(width*6/10), round(height*6/10)), 
        (round(width*5/10), round(height))] 
        ])      
    mask_curvarture_right_1 = np.zeros_like(frame)
    cv2.fillPoly(mask_curvarture_right_1, polygons_curvarture_right_1, color=(1, 1, 1)) 

    polygons_curvarture_right_2 = np.array([ 
        [(round(width*5/10), round(height)), 
        (round(width*6/10), round(height*6/10)), 
        (round(width*8/10), round(height*6/10)), 
        (round(width*10/10), round(height))] 
        ])    
    mask_curvarture_right_2 = np.zeros_like(frame)
    cv2.fillPoly(mask_curvarture_right_2, polygons_curvarture_right_2, color=(1, 1, 1)) 

    mask_all_curvarture_right = cv2.bitwise_or(mask_curvarture_right_1, mask_curvarture_right_2)

    print("=================== Start Capture =====================")
    start_frame_time = time.time()
    start_time = time.time()
    i = 0

    while True:
        # Wait for a coherent pair of frames: depth and color
        realsense_frames = pipeline.wait_for_frames()
        color_frame = realsense_frames.get_color_frame()
        if not color_frame:
            ret = 0
            continue

        ret = 1

        if ret:
            # Convert images to numpy arrays
            frame = np.asanyarray(color_frame.get_data())
            org_frame = frame

            #cv2.imshow("Org Image", cv2.resize(frame, (image_out_width, image_out_height)))

            yellow_white = extract_colour(frame)
            cv2.imshow("yellow_white Image", cv2.resize(yellow_white, (image_out_width, image_out_height)))
            
            #binary_img = cv2.Canny(yellow_white, 100, 255)
            binary_img = cv2.cvtColor(yellow_white, cv2.COLOR_BGR2GRAY)
            #cv2.imshow("Binary Image", cv2.resize(binary_img, (image_out_width, image_out_height)))

            gui_steeringLeftRight = 0
            if (gui_steeringLeftRight == 1):
                mask_left = mask_curvarture_left_1
                mask_right = mask_curvarture_left_2
                mask_all = mask_all_curvarture_left
            elif (gui_steeringLeftRight == 2):
                mask_left = mask_curvarture_right_1
                mask_right = mask_curvarture_right_2
                mask_all = mask_all_curvarture_right
            else:
                mask_left = mask_straight_left
                mask_right = mask_straight_right
                mask_all = mask_all_straight

            if (using_houghLine_flag == 1):
                cropped_left = binary_img * mask_left[:,:,1]
                #cropped_left = cv2.erode(cropped_left, erode_kernel, iterations=2)
                #cv2.imshow("ROI Left Image", cv2.resize(cropped_left, (image_out_width, image_out_height)))
                frame, x_left = hough_lines_left(cropped_left, frame)

                cropped_right = binary_img * mask_right[:,:,1]
                #cropped_right = cv2.erode(cropped_right, erode_kernel, iterations=2)
                #cv2.imshow("ROI Right Image", cv2.resize(cropped_right, (image_out_width, image_out_height)))
                frame, x_right = hough_lines_right(cropped_right, frame)
                
                if (x_left != invalid_value) and (x_right != invalid_value):
                    frame = cv2.arrowedLine(
                        img = frame, 
                        pt1 = ((x_left+x_right) // 2, height), 
                        pt2 = ((x_left+x_right) // 2, height - 20),
                        color = (0,0,0), 
                        thickness = 2,
                        tipLength = 0.3)
                    output_data.offset_houghline = ((x_left+x_right) // 2 - (width // 2)) * (width_to_meter / width)
                    output_data.houghline_available = 1
                else :
                    output_data.offset_houghline = 0
                    output_data.houghline_available = 0
            else:
                output_data.offset_houghline = 0
                output_data.houghline_available = 0
                output_img = frame

            if (using_polyfit_flag == 1):
                cropped_image = binary_img * mask_all[:,:,1]
                result_frame = polyfit_lane(cropped_image, frame, height, width)
                cv2.imshow("PolyFit Image", cv2.resize(result_frame, (image_out_width, image_out_height)))
                output_data.offset_polyfit = 0
            else:
                output_data.offset_polyfit = 0

            #frame = convertScale(frame, alpha=convertScale_alpha, beta=convertScale_beta)
            #cv2.imshow("ConvertScale Image", cv2.resize(frame, (image_out_width, image_out_height)))

            #kenerl_size = (5, 5)
            #frame = cv2.GaussianBlur(frame, kenerl_size, 0)

            #frame = frame * mask_all
            #cv2.imshow("Masked Image", cv2.resize(frame, (image_out_width, image_out_height)))

            if (using_Model_flag == 1):
                pre_prediction, prediction = predict_lane(frame)
                #cv2.imshow("Area prediction", cv2.resize(pre_prediction, (image_out_width, image_out_height)))
                #cv2.imshow("Default prediction", cv2.resize(prediction, (image_out_width, image_out_height))
                
                try:
                    con_detect_flag = True
                    output_img, para_to_calc = find_lines(pre_prediction, prediction)
                    img_size = [output_img.shape[0], output_img.shape[1]] # [height, width]
                    cal_curvature_and_offset(para_to_calc[0], para_to_calc[1], para_to_calc[2], img_size)
                    output_data.model_available = 1

                    output_img = cv2.circle(
                        img = output_img, 
                        center = (int(round(para_to_calc[2])), height), #width, height
                        radius = 3, 
                        color = (0,0,255), 
                        thickness = 2)
                    output_img = cv2.arrowedLine(
                        img = output_img, 
                        pt1 = (round(width/2), height), 
                        pt2 = (round(width/2), height - 30),
                        color = (0,0,255), 
                        thickness = 2,
                        tipLength = 0.3)
                    
                    '''
                    cv2.putText(output_img, 
                        "Good detection", 
                        org=(0, 50), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,              
                        fontScale=1, 
                        color=(255, 255, 255), 
                        thickness=2)
                    '''
                                        
                except:
                    output_data.model_available = 0
                    output_data.radius_model = 0
                    output_img = prediction
                    
                    '''
                    cv2.putText(output_img, 
                        "Bad detection", 
                        org=(0, 50), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1,
                        color=(0, 0, 255),
                        thickness=2)
                    '''
            else:
                output_data.model_available = 0
                output_data.radius_model = 0
                output_img = frame
            
            
            if (output_data.model_available == 1) and (output_data.houghline_available == 1):
                finalCenterOffset = (output_data.offset_model + output_data.offset_houghline) / 2
            elif (output_data.model_available == 1):
                finalCenterOffset = output_data.offset_model
            elif (output_data.houghline_available == 1):
                finalCenterOffset = output_data.offset_houghline
            else:
                finalCenterOffset = 0
            
            ros_msg_counter = ros_msg_counter + 1
            lane_detection_data.msg_counter = ros_msg_counter
            lane_detection_data.centerOffset = -finalCenterOffset# * 0.5
            lane_detection_data.curvature = output_data.radius_model
            pub_lane_data.publish(lane_detection_data)
            
            '''
            centerOffset_msg = "Car is {:.2f} m to the center...".format(finalCenterOffset)
            curvature_msg = "Curvature = {:.2f} km".format(output_data.radius_model)
            output_data.msg = centerOffset_msg + curvature_msg
            cv2.putText(output_img, 
                        output_data.msg, 
                        org=(0, 100), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,              
                        fontScale=1, 
                        color=(0, 0, 0), 
                        thickness=2)
            '''
            #output_img = frame
            cv2.imshow("Final result", cv2.resize(output_img, (image_out_width, image_out_height)))

            stop_frame_time = time.time()
            FPS = 1 / (stop_frame_time - start_frame_time)
            print(FPS)
            start_frame_time = stop_frame_time

        else:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pipeline.stop()
    
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

    main()
