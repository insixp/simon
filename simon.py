import numpy as np
import cv2 as cv
from mss import mss
from PIL import Image
import time
import win32api
import win32con

monitor_cap_view = {'left': 1050, 'top': 150, 'width': 800, 'height': 650}

global trigger_lock
global trigger_color
trigger_lock = False
trigger_color = None

data = ""

global count
count = 0

def wait_frames(num_of_frames):
    global count
    if(count < num_of_frames):
        count += 1
        return False
    else:
        count = 0
        return True


def click(location):
    location = [location[0] + monitor_cap_view['left'], location[1]+monitor_cap_view['top']]
    win32api.SetCursorPos(location)
    win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN,location[0], location[1] ,0,0)
    win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP,location[0] ,location[1] ,0,0)

def diff_trigger(diff, color):
    global trigger_lock
    global trigger_color
    global data
    x,y = np.where(diff >= 255)
    if(len(x) > 0):
        data += "time: {4}; color: {0}; len: {1}; trigger_lock: {2}; trigger_color: {3}\n".format(color, len(x) > 0, trigger_lock, trigger_color, time.time())
        return True
    return False

def hsv2gray(img):
    return cv.cvtColor(cv.cvtColor(img, cv.COLOR_HSV2BGR), cv.COLOR_BGR2GRAY)

def center_of_mass(threshold_img):
    mass_y, mass_x = np.where(threshold_img >= 255)
    return [(int)(np.average(mass_x)), (int)(np.average(mass_y))]

def img_to_colors(img, prev_img=None):
    img_hsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)

    mask_green = cv.inRange(img_hsv, (50, 0, 0), (80, 255, 255))
    mask_blue = cv.inRange(img_hsv, (100, 0, 0), (130, 255, 255))
    mask_red = cv.inRange(img_hsv, (0, 92, 0), (0, 174, 255))
    mask_yellow = cv.inRange(img_hsv, (5, 0, 160), (52, 255, 255))
    
    green = cv.bitwise_and(img_hsv, img_hsv, mask=mask_green)
    blue = cv.bitwise_and(img_hsv, img_hsv, mask=mask_blue)
    red = cv.bitwise_and(img_hsv, img_hsv, mask=mask_red)
    yellow = cv.bitwise_and(img_hsv, img_hsv, mask=mask_yellow)

    if not (prev_img is None):
        prev_img_hsv = cv.cvtColor(prev_img, cv.COLOR_RGB2HSV)

        prev_green = cv.bitwise_and(prev_img_hsv, prev_img_hsv, mask=mask_green)
        prev_blue = cv.bitwise_and(prev_img_hsv, prev_img_hsv, mask=mask_blue)
        prev_red = cv.bitwise_and(prev_img_hsv, prev_img_hsv, mask=mask_red)
        prev_yellow = cv.bitwise_and(prev_img_hsv, prev_img_hsv, mask=mask_yellow)

        green = cv.subtract(green, prev_green)
        blue = cv.subtract(blue, prev_blue)
        red = cv.subtract(red, prev_red)
        yellow = cv.subtract(yellow, prev_yellow)

    ret_val, green_thresh = cv.threshold(hsv2gray(green), 50, 255, cv.THRESH_BINARY)
    ret_val, blue_thresh = cv.threshold(hsv2gray(blue), 50, 255, cv.THRESH_BINARY)
    ret_val, red_thresh = cv.threshold(hsv2gray(red), 50, 255, cv.THRESH_BINARY)
    ret_val, yellow_thresh = cv.threshold(hsv2gray(yellow), 50, 255, cv.THRESH_BINARY)

    return [green_thresh, blue_thresh, red_thresh, yellow_thresh]
            
def main():
    global data
    i = 0
    with mss() as sct:
        screenshot =  sct.grab(monitor_cap_view)
        prev_img = np.array(Image.frombytes('RGB', (screenshot.width, screenshot.height), screenshot.rgb))
        green_thresh, blue_thresh, red_thresh, yellow_thresh = img_to_colors(prev_img)

        green_com = center_of_mass(green_thresh)
        blue_com = center_of_mass(blue_thresh)
        red_com = center_of_mass(red_thresh)
        yellow_com = center_of_mass(yellow_thresh)

        stage = 1
        sequence = []
        sequence_i = 0
        state = "watch"

        while True:
            screenshot =  sct.grab(monitor_cap_view)
            img_pre = np.array(Image.frombytes('RGB', (screenshot.width, screenshot.height), screenshot.rgb))
            img = cv.cvtColor(img_pre, cv.COLOR_RGB2BGR)

            green_diff, blue_diff, red_diff, yellow_diff = img_to_colors(img_pre, prev_img)

            if(state == "watch"):
                if(len(sequence) == stage):
                    state = "repeat"
                    sequence_i = 0
                else:
                    if(diff_trigger(green_diff, "g")):
                        sequence.append("g")
                    if(diff_trigger(blue_diff, "b")):
                        sequence.append("b")
                    if(diff_trigger(red_diff, "r")):
                        sequence.append("r")
                    if(diff_trigger(yellow_diff, "y")):
                        sequence.append("y")
            if(state == "repeat"):
                if(wait_frames(10)):
                    if(sequence[sequence_i] == "g"):
                        click(green_com)
                    if(sequence[sequence_i] == "b"):
                        click(blue_com)
                    if(sequence[sequence_i] == "r"):
                        click(red_com)
                    if(sequence[sequence_i] == "y"):
                        click(yellow_com)
                    sequence_i += 1

                if(sequence_i == stage):
                    if(wait_frames(3)):
                        stage += 1
                        sequence = []
                        state = "watch"
            all_diff = cv.bitwise_or(cv.bitwise_or(cv.bitwise_or(green_diff, blue_diff), red_diff), yellow_diff)  
            i+=1
            prev_img = img_pre

            cv.namedWindow("control_window")

            if(cv.waitKey(1) & 0xFF == ord('q')):
                cv.destroyAllWindows()
                return



if __name__ == "__main__":
    main()