import cv2
import numpy as np
import math
import random as r

fps = 10
delay = int(1000/fps)
horizontal = [0,320,640,960]
vertical = [0,320,640,960,1280]

def capture_background():
    capture = cv2.VideoCapture(0)

    print("Press B to capture background.")

    while True:
        ret, frame = capture.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (1280,960))
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(delay)
        if key == ord('b') or key == ord('B'):
            background = frame
            break

    cv2.destroyAllWindows()
    capture.release()
    print("Background captured.")

    return frame

def draw_areas(img):
    for h in horizontal:
        cv2.line(img, (0, h), (1280, h), (0,255,0), 1, cv2.LINE_AA)
    
    for v in vertical:
        cv2.line(img, (v, 0), (v, 960), (0,255,0), 1, cv2.LINE_AA)

    return img

def draw_text(img, finger, current_finger, score):
    cv2.putText(img, f"(debug)Current fingers: {current_finger}", (0, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    cv2.putText(img, f"Show {finger} fingers", (0, 110), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    cv2.putText(img, f"Score: {score}", (0, 160), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    return img

def select_area():
    v = r.randint(0,2) * 320
    h = r.randint(0,3) * 320
    return [h,v]

def create_foreground_mask(roi, background):
    diff = cv2.subtract(roi, background) + cv2.subtract(background, roi)
    diff[abs(diff) < 25.0] = 0
    fg_mask = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    fg_mask[np.abs(fg_mask) < 10] = 0
    kernel = np.ones((5,5), np.uint8)
    fg_mask = cv2.erode(fg_mask, kernel, iterations=2)
    fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)
    fg_mask[np.abs(fg_mask) > 5] = 255

    return fg_mask

def calculate_finger_count(img, roi):
    try:
        contours, hierarchy = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = max(contours, key = lambda x: cv2.contourArea(x))
        epsilon = 0.0005*cv2.arcLength(cnt,True)
        approx= cv2.approxPolyDP(cnt,epsilon,True)
       
        
        hull = cv2.convexHull(cnt)

        areahull = cv2.contourArea(hull)
        areacnt = cv2.contourArea(cnt)
      
        arearatio=((areahull-areacnt)/areacnt)*100

        hull = cv2.convexHull(approx, returnPoints=False)
        defects = cv2.convexityDefects(approx, hull)
        
        l=0
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(approx[s][0])
            end = tuple(approx[e][0])
            far = tuple(approx[f][0])
            
            
            a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
            c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
            s = (a+b+c)/2
            ar = math.sqrt(s*(s-a)*(s-b)*(s-c))
            
            d=(2*ar)/a
            
            angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
            
        
            if angle <= 90 and d>30:
                l += 1
            
            
        l+=1
        if l == 1:

            if areacnt<2000:
                return None
            else:
                if arearatio < 12:
                    return 0
                else:
                    return 1
        elif l > 5:
            return None
        else:
            return l
    except:
        return None
    
def videoCapture(background):
    capture = cv2.VideoCapture(0)
    finger_count = 0,
    score = 0
    finger = r.randint(0, 5)
    area = select_area()
    print(area)
    while True:
        ret, frame = capture.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (1280,960))
        
        
        roi = frame[area[1]:area[1]+320, area[0]:area[0]+320]
        

        fg_mask = create_foreground_mask(roi, background[area[1]:area[1]+320, area[0]:area[0]+320])
        finger_count = calculate_finger_count(frame, fg_mask)
        frame = draw_text(frame, finger, finger_count, score)
        frame = draw_areas(frame)
        cv2.rectangle(frame, (area[0],area[1]), (area[0]+320,area[1]+320),(0,0,255), 3, cv2.LINE_AA)
        key = cv2.waitKey(delay)

        if key == ord('n') or key == ord('N'):
            if finger == finger_count:
                score+=1
            finger = r.randint(0,5)
            area = select_area()

        cv2.imshow("Foreground mask", fg_mask)
        cv2.imshow("Capture", frame)
        cv2.imshow("(debug)roi", roi)

        if key == ord('q') or key == ord('Q'):
            break
        
        
        
        

    cv2.destroyAllWindows()
    capture.release()


background = capture_background()
videoCapture(background)
