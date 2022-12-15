import time
import cv2
import threading
import numpy as np
from Arm_Lib import Arm_Device
import speech_recognition as sr  

Arm = Arm_Device()


    

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
    



class camThread(threading.Thread):
    def __init__(self, colorinput, previewName, camID):
        threading.Thread.__init__(self)
        self.colorinput = colorinput
        self.previewName = previewName
        self.camID= camID
        
        
    def run(self):
        print('starting ',self.previewName)
        #toolboxCam(self.colorinput,self.previewName,self.camID)
        toolboxCam(self.colorinput, self.previewName,self.camID)


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
    #driveForward(0,0)
    pass
    
def destroy():
    Car_Stop()

    cap.release()
    cv2.destroyAllWindows()
    

    
    
    
if __name__ == '__main__':

    
    #mic= sr.Microphone()
    commandLibrary = ['green', 'yellow','red','exit','release']
    colorLibrary = ['green', 'yellow','red']
    initialPos()
    

    camID =input('Enter your camera id: ')
    camID= int(camID)
    


    if camID == 0:
        thread1 = camThread('yellow',"Camera1", camID)
        thread1.start()
        time.sleep(0.1)


    if camID == 2:
        thread2 = camThread('yellow',"Camera2", camID)
        thread2.start()
        time.sleep(0.1)

    
  
            

