import cv2
import numpy as np
import math
import random as r
import RPi.GPIO as GPIO
import time
from os import system

GPIO.setmode(GPIO.BCM)
# Setting Up Buttons
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Left Button
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Right Button
# Setting Up 7-Segment LED
GPIO.setup(9, GPIO.OUT) # A 
GPIO.setup(11, GPIO.OUT)  # B
GPIO.setup(5, GPIO.OUT) # C
GPIO.setup(6, GPIO.OUT) # D
GPIO.setup(13, GPIO.OUT) # E
GPIO.setup(19, GPIO.OUT) # F
GPIO.setup(26, GPIO.OUT) # G
dat = [0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F] # 7 Segment numbers
# Setting Up LEDs
GPIO.setup(24, GPIO.OUT) # Red Led 
GPIO.setup(23, GPIO.OUT) # Green Led

fps = 15
rate_of_calc = 15
delay = int(1000/fps)
res_mult = 100
horizontal = [0,res_mult*1,res_mult*2,res_mult*3]
vertical = [0,res_mult*1,res_mult*2,res_mult*3,res_mult*4]
diff_sensivity = 25.0 # higher = less sensivity
bg_sensivity = int(diff_sensivity) # keep it same as diff_sensivity

def PORT(pin): # Showing the values in 7 Segment.
    if(pin&0x01 == 0x01):
        GPIO.output(9,0)            # if  bit0 of 8bit 'pin' is true, pull PIN13 high
    else:
        GPIO.output(9,1)            # if  bit0 of 8bit 'pin' is false, pull PIN13 low
    if(pin&0x02 == 0x02):
        GPIO.output(11,0)             # if  bit1 of 8bit 'pin' is true, pull PIN6 high
    else:
        GPIO.output(11,1)            #if  bit1 of 8bit 'pin' is false, pull PIN6 low
    if(pin&0x04 == 0x04):
        GPIO.output(5,0)
    else:
        GPIO.output(5,1)
    if(pin&0x08 == 0x08):
        GPIO.output(6,0)
    else:
        GPIO.output(6,1)   
    if(pin&0x10 == 0x10):
        GPIO.output(13,0)
    else:
        GPIO.output(13,1)
    if(pin&0x20 == 0x20):
        GPIO.output(19,0)
    else:
        GPIO.output(19,1)
    if(pin&0x40 == 0x40):
        GPIO.output(26,0)
    else:
        GPIO.output(26,1)

def PORT2(pin): # Showing the values in 7 Segment.
    if(pin&0x01 == 0x01):
        GPIO.output(25,1)            # if  bit0 of 8bit 'pin' is true, pull PIN13 high
    else:
        GPIO.output(25,0)            # if  bit0 of 8bit 'pin' is false, pull PIN13 low
    if(pin&0x02 == 0x02):
        GPIO.output(8,1)             # if  bit1 of 8bit 'pin' is true, pull PIN6 high
    else:
        GPIO.output(8,0)            #if  bit1 of 8bit 'pin' is false, pull PIN6 low
    if(pin&0x04 == 0x04):
        GPIO.output(7,1)
    else:
        GPIO.output(7,0)
    if(pin&0x08 == 0x08):
        GPIO.output(12,1)
    else:
        GPIO.output(12,0)   
    if(pin&0x10 == 0x10):
        GPIO.output(16,1)
    else:
        GPIO.output(16,0)
    if(pin&0x20 == 0x20):
        GPIO.output(20,1)
    else:
        GPIO.output(20,0)
    if(pin&0x40 == 0x40):
        GPIO.output(21,1)
    else:
        GPIO.output(21,0)

def capture_background():
    system('clear')
    capture = cv2.VideoCapture(1)

    print("Press Left Button to capture background.")

    while True:
        _, frame = capture.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (res_mult*4,res_mult*3))
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(delay)
        button_state = GPIO.input(17)
        if button_state == False: # Arkaplan yakalanması için butona basıldı.
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
                return 0
            else:
                if arearatio < 12:
                    return 0
                else:
                    return 1
        elif l > 5:
            return 0
        else:
            return l
    except:
        return 0
    
def videoCapture(background):
    system('clear')
    capture = cv2.VideoCapture(1)
    finger_count = 0
    pin = dat[finger_count] # Initial state (0) shown in 7 Segment Display
    PORT(pin)
    score = 0
    # TODO 
    #pin = dat[score] # Initial score (0) shown in 7-Seg Disp.
    #PORT2(pin) 
    cur_frame = 0
    show_finger = r.randint(1, 5)
    area = select_area()
    while True:
        cur_frame += 1
        _, frame = capture.read()
        frame = cv2.resize(frame, (res_mult*4,res_mult*3))
        frame = cv2.flip(frame, 1)
        cv2.putText(frame,f"Show {show_finger} fingers.",(0,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2,cv2.LINE_AA)
        
        roi = frame[area[1]:area[1]+res_mult, area[0]:area[0]+res_mult]

        if cur_frame == rate_of_calc: # saniyede bir defa dönüyor
            
            GPIO.output(23, False)#yeşil ışık söndürüldü
            GPIO.output(24,False)#KIRMIZI IŞIK
            fg_mask = create_foreground_mask(roi, background[area[1]:area[1]+res_mult, area[0]:area[0]+res_mult])
            finger_count = calculate_finger_count(fg_mask)
            pin = dat[finger_count] # Finger count shown in 7 Segment
            PORT(pin)
            cur_frame = 0
            cv2.imshow("(debug)Foreground mask", fg_mask)
            print(f"(debug)Current fingers: {finger_count}\n")
            
        frame = draw_areas(frame, area)
        key = cv2.waitKey(delay)
        button_state = GPIO.input(27)
        if button_state == False:
            
            #BİRİSİ score, DİĞERİ finger_count gösterecek

            if show_finger == finger_count: 
                score+=1
                pin = dat[score] # Score in second Seven Segment
                GPIO.output(23, True)#YEŞİL IŞIK
                #score 7 seg sürülecek
            else:
                GPIO.output(24,True)#KIRMIZI IŞIK
            show_finger = r.randint(1,5)
            area = select_area()
            system('clear')
            cur_frame = 0;
        
        cv2.imshow("Capture", frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        button_state = GPIO.input(17)
        if button_state==False:
            break
        
        
    
    #7SEGMENTLERİ SIFIRLA
    GPIO.cleanup()
    cv2.destroyAllWindows()
    capture.release()


background = capture_background()
videoCapture(background)
