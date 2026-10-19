import cv2
import numpy as np
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))
def is_col(hsv, x, y, col):
    h, s, v = hsv[y, x]
    if col == 0: # yellow
        return (15 <= h <= 35) and (s >= 150)
    elif col == 1: # red
        return (h <= 10 or h >= 170) and (s >= 150)
    elif col == 2: # blue
        return (100 <= h <= 130) and (s >= 150)
    else:
        return False
def runPipeline(image, llrobot):
    colmode = llrobot[0] if len(llrobot or []) > 0 else 0
    contours = []
    centers = []
    # image = cv2.bilateralFilter(image,13,sigmaColor=75, sigmaSpace=75)
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)

    # Single conversion pass
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    l = lab[:, :, 0]
    a = lab[:, :, 1]
    b = lab[:, :, 2]
    # h, s, v = cv2.split(hsv)
    # a = cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX)
    # b = cv2.normalize(b, None, 0, 255, cv2.NORM_MINMAX)
    darkness_mask = cv2.inRange(l, 30, 255)
    yellow = cv2.addWeighted(a, 0.3, b, 0.7, 0)
    red = a
    blue = 255-b
    cols = [yellow, red, blue]
    masked = cv2.bitwise_and(cols[colmode], cols[colmode], mask=darkness_mask)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)

    circles = cv2.HoughCircles(
        masked,
        cv2.HOUGH_GRADIENT,
        dp=2.0,
        minDist=100,
        param1=60,
        param2=80,
        minRadius=20,
        maxRadius=140
    )
    if circles is not None:
        circles = np.uint16(np.around(circles[0, :5]))
        for x, y, r in circles:
            # Clip to bounds
            x = np.clip(x, 0, 639)
            y = np.clip(y, 0, 479)
            
            # Check center point
            h, s, v = hsv[y, x]
            is_color = is_col(hsv, x, y, colmode)
            
            # Check offset point only if center not yellow
            if not is_color:
                x_off = np.clip(x + r//2, 0, 639)
                y_off = np.clip(y + r//3, 0, 479)
                h, s, v = hsv[y_off, x_off]
                is_color = is_col(hsv, x_off, y_off, colmode)

            if(is_color):
                cv2.circle(image, (x, y), r, (0, 255, 255), 2)
                
                centers.append([x,y,r])

    #print([entry for center in centers for entry in center])
    return contours, image, [entry for center in centers for entry in center]