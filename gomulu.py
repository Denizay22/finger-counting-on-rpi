import cv2
import numpy as np
import math
import random as r
from os import system
fps = 15
rate_of_calc = fps / 3
delay = int(1000/fps)
res_mult = 100
horizontal = [0,res_mult*1,res_mult*2,res_mult*3]
vertical = [0,res_mult*1,res_mult*2,res_mult*3,res_mult*4]
diff_sensivity = 25.0 # higher = less sensivity
bg_sensivity = int(diff_sensivity) # keep it same as diff_sensivity

def capture_background():
    system('clear')
    capture = cv2.VideoCapture(1)

    print("Press B to capture background.")

    while True:
        _, frame = capture.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (res_mult*4,res_mult*3))
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(delay)
        if key == ord('b') or key == ord('B'):
            break

    cv2.destroyAllWindows()
    capture.release()
    print("Background captured.")

    return frame

def draw_areas(img, area):
    for h in horizontal:
        cv2.line(img, (0, h), (res_mult*4, h), (0,255,0), 1, cv2.LINE_AA)
    
    for v in vertical:
        cv2.line(img, (v, 0), (v, res_mult*3), (0,255,0), 1, cv2.LINE_AA)

    cv2.rectangle(img, (area[0],area[1]), (area[0]+res_mult,area[1]+res_mult),(0,0,255), 3, cv2.LINE_AA)
    return img

def draw_text(finger, current_finger):
    #cv2.putText(img, f"(debug)Current fingers: {current_finger}", (0, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    #cv2.putText(img, f"Show {finger} fingers", (0, 110), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    #cv2.putText(img, f"Score: {score}", (0, 160), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    #cv2.putText(img, f"Frame: {cur_frame}", (0, 160), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 3, cv2.LINE_AA)
    print(f"(debug)Current fingers: {current_finger}\n" + f"Show {finger} fingers\n")
    #return img

def select_area():
    v = r.randint(0,2) * res_mult
    h = r.randint(0,3) * res_mult
    return [h,v]

def create_foreground_mask(roi, background):
    diff = cv2.subtract(roi, background) + cv2.subtract(background, roi)
    diff[abs(diff) < diff_sensivity] = 0
    fg_mask = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    #fg_mask = cv2.subtract(roi, background) + cv2.subtract(background, roi)
    fg_mask[np.abs(fg_mask) < bg_sensivity] = 0
    kernel = np.ones((5,5), np.uint8)
    fg_mask = cv2.erode(fg_mask, kernel, iterations=2)
    fg_mask = cv2.dilate(fg_mask, kernel, iterations=2)
    fg_mask[np.abs(fg_mask) > 5] = 255

    return fg_mask

def calculate_finger_count(roi):
    try:
        contours, _ = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
    system('clear')
    capture = cv2.VideoCapture(1)
    finger_count = 0
    score = 0
    cur_frame = 0
    show_finger = r.randint(0, 5)
    area = select_area()
    while True:
        cur_frame += 1
        _, frame = capture.read()
        frame = cv2.resize(frame, (res_mult*4,res_mult*3))
        #gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.flip(frame, 1)
        
        
        
        roi = frame[area[1]:area[1]+res_mult, area[0]:area[0]+res_mult]

        if cur_frame == rate_of_calc: # doing every 3rd frame to save some cpu
            fg_mask = create_foreground_mask(roi, background[area[1]:area[1]+res_mult, area[0]:area[0]+res_mult])
            finger_count = calculate_finger_count(fg_mask)
            cur_frame = 0
            draw_text(show_finger, finger_count)
            cv2.imshow("(debug)Foreground mask", fg_mask)
            
            #cv2.imshow("(debug)roi", roi)
            

        #frame = draw_text(frame, finger, finger_count, score, cur_frame) # using console to show instead of drawing on frame to save some cpu
        
        frame = draw_areas(frame, area)
        key = cv2.waitKey(delay)

        if key == ord('n') or key == ord('N'):
            # skor sistemi yerine, 7segmentte kaç parmak göstermesi gerektiği, led varsa da doğru yaptığında led yanması ayarlanabilir
            # eğer 2 tane 7segment varsa birinde skor, birinde kaç parmak gösterdiği gösterilebilir
            if show_finger == finger_count:
                score+=1
            show_finger = r.randint(0,5)
            area = select_area()
            system('clear')

        
        cv2.imshow("Capture", frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if key == ord('q') or key == ord('Q'):
            break
        
        
        
        

    cv2.destroyAllWindows()
    capture.release()


background = capture_background()
videoCapture(background)
