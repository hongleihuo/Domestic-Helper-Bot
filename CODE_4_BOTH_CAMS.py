import cv2
import time
import threading

class camThread(threading.Thread):
    def __init__(self, previewName, camID):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.camID= camID
        
    def run(self):
        print('starting ',self.previewName)
        camPreview(self.previewName,self.camID)
        
    
def camPreview(previewName, camID):
    cv2.namedWindow(previewName)
    cam = cv2.VideoCapture(camID)
    
    if cam.isOpened():
        rval, frame = cam.read()
        
    else:
        rval= False
        
    while rval:
        cv2.imshow(previewName, frame)
        rval, frame = cam.read()
        key = cv2.waitKey(20)
        if key == 27:
            break
    cv2.destroyWindow(previewName)
    
    
camID =input('Enter your camera id: ')
camID= int(camID)
#while camID !='exit':
if camID == 0:
    thread1 = camThread("Camera1", camID)
    thread1.start()
    time.sleep(0.1)


if camID == 2:
    thread2 = camThread("Camera2", camID)
    thread2.start()
    time.sleep(0.1)




