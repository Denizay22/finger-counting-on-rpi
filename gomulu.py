import cv2
import numpy as np
import math
import random as r
import RPi.GPIO as GPIO
import time
from os import system


class SegmentedDisplay:
    NUMBER = [0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F] # 7 Segment numbers
    MASK = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40]

    def __init__(self, pins: list):
        self.pins = pins
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)

    def display(self, n):
        data = self.NUMBER[n]

        for mask, pin in zip(self.MASK, self.pins):
            if(data & mask == mask):
                GPIO.output(pin, 0)
            else:
                GPIO.output(pin, 1)

    def blank(self):
        for pin in self.pins:
            GPIO.output(pin, 1)


class Button:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read(self):
        return GPIO.input(self.pin)


class LED:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)

    def set(self, state):
        GPIO.output(self.pin, state)

    def on(self):
        self.set(True)

    def off(self):
        self.set(False)

GPIO.setmode(GPIO.BCM)

# Setting Up Buttons
left_button = Button(17)
right_button = Button(27)

# Setting Up 7-Segment LED
finger_display = SegmentedDisplay([9,11,5,6,13,19,26])
score_display = SegmentedDisplay([25,8,7,12,16,20,21])

finger_display.display(0)
score_display.display(0)

# Setting Up LEDs
red_led = LED(24)
green_led = LED(23)


fps = 15
rate_of_calc = 15
delay = int(1000/fps)
res_mult = 240
horizontal = [0,res_mult*1,res_mult*2]
vertical = [0,res_mult*1,res_mult*2]
diff_sensivity = 30.0 # higher = less sensivity
bg_sensivity = int(diff_sensivity) # keep it same as diff_sensivity

def capture_background():
    system('clear')
    capture = cv2.VideoCapture(1)

    print("Press Left Button to capture background.")

    while True:
        _, frame = capture.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (res_mult*2,res_mult*2))
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(delay)
        if not left_button.read(): # Arkaplan yakalanması için butona basıldı.
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
    v = r.randint(0,1) * res_mult
    h = r.randint(0,1) * res_mult
    return [h,v]

def create_foreground_mask(roi, background):
    diff = cv2.subtract(roi, background) + cv2.subtract(background, roi)
    diff[abs(diff) < diff_sensivity] = 0
    fg_mask = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
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
    score = 0
    cur_frame = 0
    show_finger = r.randint(1, 5)
    area = select_area()
    while True:
        cur_frame += 1
        _, frame = capture.read()
        frame = cv2.resize(frame, (res_mult*2,res_mult*2))
        frame = cv2.flip(frame, 1)
        cv2.putText(frame,f"Show {show_finger} fingers.",(0,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2,cv2.LINE_AA)
        
        roi = frame[area[1]:area[1]+res_mult, area[0]:area[0]+res_mult]

        if cur_frame == rate_of_calc: # saniyede bir defa dönüyor
            
            green_led.off()
            red_led.off()
            fg_mask = create_foreground_mask(roi, background[area[1]:area[1]+res_mult, area[0]:area[0]+res_mult])
            finger_count = calculate_finger_count(fg_mask)
            finger_display.display(finger_count)
            cur_frame = 0
            cv2.imshow("(debug)Foreground mask", fg_mask)
            print(f"(debug)Current fingers: {finger_count}\n")
            
        frame = draw_areas(frame, area)
        key = cv2.waitKey(delay)
        if not right_button.read():
            
            #BİRİSİ score, DİĞERİ finger_count gösterecek

            if show_finger == finger_count: 
                score+=1
                score_display.display(score)
                green_led.on()
                red_led.off()
            else:
                red_led.on()
            show_finger = r.randint(1,5)
            area = select_area()
            system('clear')
            cur_frame = 0;
            cv2.waitKey(1000)
        
        cv2.imshow("Capture", frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if not left_button.read(): # Arkaplan yakalanması için butona basıldı.
            break
        
        
    
    #7SEGMENTLERİ SIFIRLA
    GPIO.cleanup()
    cv2.destroyAllWindows()
    capture.release()


background = capture_background()
videoCapture(background)
