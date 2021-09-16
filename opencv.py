import numpy as np
import cv2 as cv
from mss import mss
from PIL import Image

monitor_cap_view = {'left': 0, 'top': 0, 'width': 800, 'height': 800}

with mss() as sct:
    while True:
        screenshot =  sct.grab(monitor_cap_view)
        img = np.array(Image.frombytes('RGB', (screenshot.width, screenshot.height), screenshot.rgb))
        gray_scale = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        img_blur = cv.GaussianBlur(gray_scale, (3, 3), 0)
        edges = cv.Canny(image=img_blur, threshold1=100, threshold2=200)        
        cv.imshow('screen', edges)

        if(cv.waitKey(1) & 0xFF == ord('q')):
            cv.destroyAllWindows()
            break