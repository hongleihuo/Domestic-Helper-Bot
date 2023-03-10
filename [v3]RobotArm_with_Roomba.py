#!/usr/bin/env python
# coding: utf-8

# In[1]:


import time
import cv2
import numpy as np
import threading
from Arm_Lib import Arm_Device
import speech_recognition as sr 
from pycreate2 import Create2
import time
import serial


######################## Initialize #######################################

# initialize variables and ports
port = "/dev/ttyUSB0"
bot = Create2(port)
ser = serial.Serial(port, baudrate = 115200)
#sensors = bot.get_sensors()
sensors =''
wheel2center = 113.5
wheelSpeed = 200
turnSpeed = 50
forwardSpeed = 100
singleWheelTime = 3.1415926 * wheel2center / wheelSpeed
tolerance = singleWheelTime * 0.05
singleWheelTime = singleWheelTime + tolerance
move = 0


# Initialize for arm
Arm = Arm_Device()
x_bias_roomba = 130
rad_bias_roomba = 80
xmin_roomba = 314 - x_bias_roomba
xmax_roomba = 326 + x_bias_roomba


########################### Roomba Funcitons ####################################

def displayBatteryLife(flag):
    if flag == 1:
        print('battery', sensors.battery_capacity*100/3000,'%')
    else:
        print(' ')
def beep(song_num):
    Song = []
    song0 = [59, 64, 62, 32, 69, 96, 67, 64, 62, 32, 60, 96, 59, 64, 59, 32, 59, 32, 60, 32, 62, 32, 64, 96, 62, 96]
    song1 = [76, 16, 76, 16, 76, 32, 76, 16, 76, 16, 76, 32, 76, 16, 79, 16, 72, 16, 74, 16, 76, 32, 77, 16, 77, 16, 77, 16, 77, 32, 77, 16]
    song2 = [76, 12, 76, 12, 20, 12, 76, 12, 20, 12, 72, 12, 76, 12, 20, 12, 79, 12, 20, 36, 67, 12, 20, 36]
    song3 = [72, 12, 20, 24, 67, 12, 20, 24, 64, 24, 69, 16, 71, 16, 69, 16, 68, 24, 70, 24, 68, 24, 67, 12, 65, 12, 67, 48]
    Song.append(song0)
    Song.append(song1)
    Song.append(song2)
    Song.append(song3)

    # song number can be 0-3
    bot.createSong(song_num, Song[song_num])
    time.sleep(0.1)
    songDuration = bot.playSong(song_num)
    time.sleep(songDuration)
    
    
def getbumpSensor():
    ser.write(b'\x8E\x07')
    time.sleep(0.2)
    while ser.inWaiting()!=0:
        response = []
        response.append(hex(ord(ser.read())))
        if int(response[0],16)>0:
            return 1
        else:
            return 0

def driveForward(forwardSpeed,t):
    #roomba travels straight
    bot.drive_direct(forwardSpeed,forwardSpeed)
    time.sleep(t)
    
    
def driveBackward(forwardSpeed):
    #roomba travels straight
    bot.drive_direct(-forwardSpeed,-forwardSpeed)
    time.sleep(0.5)
    
def Uturn(wheelSpeed):
    bot.drive_direct(wheelSpeed, -wheelSpeed)
    time.sleep(singleWheelTime)
    
def leftTurn(turnSpeed,t):
    bot.drive_direct(turnSpeed, -turnSpeed)
    time.sleep(t)
    
def rightTurn(turnSpeed, t):
    bot.drive_direct(-turnSpeed,turnSpeed)
    time.sleep(t)





########################### Robot Arm Funcitons ####################################

def initialPos():
    
    #initial position
    look_at = [90, 135, 0, 0, 90, 30]
    arm_move_6(look_at, 1000)
    time.sleep(1)

def arm_move_6(p, s_time = 500):
    for i in range(6):
        id = i + 1
        Arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)

  


def grab():
#                     5    4   3   2 
    servoAngle2_5 = [270, 75, 60 , 15]
    count = 5
    for angle in servoAngle2_5: 
        print(count, angle)
        Arm.Arm_serial_servo_write(count, angle, 1000)
        time.sleep(1)
        count -=1
    Arm.Arm_serial_servo_write(6, 180, 1000)      

def passing():
     
 
    tick = 2
 #                2   3   4  5   6
    servoAngle = [90, 90, 60, 90, 180]
    for ang in servoAngle:
        print(tick, ang)
        Arm.Arm_serial_servo_write(tick, ang, 1000)
        time.sleep(1)
        tick +=1
        
def release():
    
    Arm.Arm_serial_servo_write(6, 0, 1000)
    time.sleep(1)
    
                            
########################### Roomba + Robot Arm Funciton(s) ####################################

class camThread(threading.Thread):
    def __init__(self, colorinput, previewName, camID):
        threading.Thread.__init__(self)
        self.colorinput = colorinput
        self.previewName = previewName
        self.camID= camID
        
        
    def run(self):
        print('starting ',self.previewName)
        toolboxCam(self.colorinput,self.previewName,self.camID)
        
def toolboxCam(colorinput, previewName, camID):
    #cv2.namedWindow(previewName)
    cap = cv2.VideoCapture(camID)
    camID = int(camID)
    if camID  == 0:
        print('robot arm camera is activated')
        initial_angle  = 90
        x_bias = 130
        rad_bias = 30
        x_now = 0
        x_mid= 320
        rad_now = 0
        rad_mid = 70
        control_flag = 0
        xmin = 314
        xmax = 326
    
    if camID  == 2:
        print('roomba camera is activated')
        initial_angle  = 90
        x_bias = 130
        rad_bias = 30
        x_now = 0
        x_mid= 320
        rad_now = 0
        rad_mid = 70
        control_flag = 0
        xmin = 314
        xmax = 326
        
    while True:
        #cv2.imshow(previewName, frame)
        rval, frame = cap.read()
        frame = cv2.rectangle(frame, (260,180), (380, 300),(255, 255, 255), 1)
        frame = cv2.rectangle(frame, (xmin,0), (xmax,480),(255,255,255),1)
        ########################image processing###########################
        Gaussian = cv2.GaussianBlur(frame, (5, 5), 0)
        hsvspace = cv2.cvtColor(Gaussian, cv2.COLOR_BGR2HSV)
        lower = np.zeros(3)
        upper = np.zeros(3)
        
        if colorinput  == 'green':
            lower = np.array([35, 43, 35])
            upper = np.array([90, 255, 255])
            
        
        if colorinput =='yellow':
            lower = np.array([25, 50, 70])
            upper = np.array([35, 255, 255])
           

        if colorinput =='red':
            lower = np.array([0, 50, 70])
            upper = np.array([9, 255, 255])
            

        whiteandblack = cv2.inRange(hsvspace, lower, upper)
        kernal = np.ones((3,3))
        smoothresult = cv2.erode(whiteandblack, kernal, iterations=2)
        contour = cv2.findContours(smoothresult.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        if len(contour) > 0:
            area = max(contour, key=cv2.contourArea)
            (x, y), rad = cv2.minEnclosingCircle(area)
            if int(rad) > 20 and int(rad) < 200:
                x_now = x
                rad_now = rad
                print('x: ', int(x_now), 'y :', int(y))
                
                
    ############################control##################################
                if camID == 0:
                    if x_now  < xmin:
                        print('lefttward!')
                        
                        Arm.Arm_serial_servo_write(1, initial_angle, 1000)
                        initial_angle+= 0.5
                        time.sleep(0.5)
                        
                        
                    elif x_now > xmax:
                        
                        #initial_angle -=1
                        print('rightward!')
                        Arm.Arm_serial_servo_write(1, initial_angle, 1000)
                        initial_angle-=0.5
                        time.sleep(0.5)
                        
                    
                    elif x_now <xmax and x_now > xmin:
                        print('grab the object')
                        grab()
                        time.sleep(1)
                        print('passing the object')
                        passing()
                        time.sleep(1)
                        break
                            
                        
                        
                if camID == 2:
                        if x_now  < xmin:
                            print('turn left')
                            leftTurn(50, 0.1)
                            
                            
                        elif x_now > xmax:
                            print('turn right')
                            rightTurn(50, 0.1)
                            
                            
                        
                        elif x_now <xmax and x_now > xmin:
                            print('driving forward')
                            driveForward(forwardSpeed, 5)
                        
                            
            if int(rad)>200 and camID == 2:
                    print('robot is getting close to the toolbox')
                    #driveForward(forwardSpeed,5)
                            
                        
            else:
                time.sleep(1)
                print('Nothing detect!')
                Car_Stop()
        else:
            pass
                
            
        ####################For Testing########################################
        cv2.imshow('frame', frame)
        #cv2.imshow('Gaussian',Gaussian)
        #cv2.imshow('whiteandblack',whiteandblack)
        #cv2.imshow('smoothresult',smoothresult)
            
        key = cv2.waitKey(20)
        if key == 27:
            break   
      
    cap.release()
    cv2.destroyAllWindows()
######################shut down everything, close windows#########################
def Car_Stop():
    driveForward(0,0) 

def destroy():
    Car_Stop()
    cap.release()
    cv2.destroyAllWindows()
######################### Main Function ################################
    
    
if __name__ == '__main__':

    # start the robot
    #bot.start()
    #bot.safe()
    #time.sleep(0.1)
    #beep(2)
    
    # start the voice recognition
    #r =sr.Recognizer()  
    #r.energy_threshold = 6000
    
    #mic= sr.Microphone()
    commandLibrary = ['exit','release','toolbox']
    colorLibrary = ['green', 'yellow','red']
    time.sleep(1)
    initialPos()
    time.sleep(1)
    
    
    initialPos()
    userinput = 'toolbox'
            
    if userinput in commandLibrary:
        if userinput =='toolbox':
            toolbox_color ='yellow'
            Arm.Arm_Buzzer_On(1)
            camID = 0
            previewName =' Camera2'
            thread1 = camThread(toolbox_color ,previewName, camID)
            thread1.start()
            time.sleep(0.1)
            
        if userinput =='release':
            Arm.Arm_Buzzer_On(1)
            release()
            time.sleep(10)
            initialPos()            
            
        if userinput =='exit':
            Arm.Arm_Buzzer_On(1)
            initialPos()
            
            
    else:
        print('Enter the correct input: ', commandLibrary)
        #audio = r.listen(source) 
        
    print('system completed')
    
    
    program = True
    
        
    while program == False:
        #uncomment below for voice control
        #with mic as source:
            #r.adjust_for_ambient_noise(source,duration =0.5)
            #print('speak')
            #audio = r.listen(source)
            #time.sleep(0.1)  
            
        try:
            #print("You said " + r.recognize_google(audio, language ='en-US'))
            #userinput = str(r.recognize_google(audio))
            #userinput = userinput.lower()
            
            #userinput = input('Enter your command')
            #colorinput = input('Enter color')
            userinput = 'toolbox'
            
            if userinput in commandLibrary:
                if userinput =='toolbox':
                    toolbox_color ='yellow'
                    Arm.Arm_Buzzer_On(1)
                    camID = 0
                    previewName =' Camera2'
                    thread1 = camThread(toolbox_color ,previewName, camID)
                    thread1.start()
                    time.sleep(0.1)
                    
                if userinput =='release':
                    Arm.Arm_Buzzer_On(1)
                    release()
                    time.sleep(10)
                    initialPos()            
                    
                if userinput =='exit':
                    Arm.Arm_Buzzer_On(1)
                    initialPos()
                    break
                    
            else:
                print('Enter the correct input: ', commandLibrary)
                audio = r.listen(source) 
                
            print('system completed')
        except sr.UnknownValueError:
            print("Could not understand audio")
        except sr.RequestError as e:
            print("Could not request results; {0}".format(e))
        except KeyboardInterrupt:
            destroy()


# In[ ]:





    
    
    
    
    
   