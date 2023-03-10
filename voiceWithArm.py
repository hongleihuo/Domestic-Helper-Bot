from Arm_Lib import Arm_Device
import time

Arm = Arm_Device()
def arm_move_6(p, s_time = 500):
    for i in range(6):
        id = i + 1
        Arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(.01)
    time.sleep(s_time/1000)
    

look_at = [90, 90, 25, 25, 90, 30]
arm_move_6(look_at, 1000)
time.sleep(1)


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

def Passing():
     
 
    tick = 2
 #                2   3   4  5   6
    servoAngle = [90, 90, 60, 90, 180]
    for ang in servoAngle:
        print(tick, ang)
        Arm.Arm_serial_servo_write(tick, ang, 1000)
        time.sleep(1)
        tick +=1
       
 


import speech_recognition as sr  

# get audio from the microphone                                                                       
r =sr.Recognizer()                                                                                   
with sr.Microphone() as source: 
    r.adjust_for_ambient_noise(source,duration =1)                                                                      
    print("Speak:")                                                                                   
    audio = r.listen(source)   
    
try:
    print("You said " + r.recognize_google(audio, language ='en-US'))
    voiceInput = str(r.recognize_google(audio))
    
    if voiceInput.lower() == 'start':
            grab()
            Passing()
            Arm.Arm_serial_servo_write(2, 90, 100)
            
    
except sr.UnknownValueError:
    print("Could not understand audio")
except sr.RequestError as e:
    print("Could not request results; {0}".format(e))
    
    
    
    

