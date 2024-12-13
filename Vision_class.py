import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
from ultralytics import YOLO

class Vision():
    
    def __init__(self):
        self.cap = cv.VideoCapture(2)
        self.conf_threshold = 0.80
        self.crop_y1 = 180
        self.crop_y2 = 350
        self.crop_x1 = 200
        self.crop_x2 = 500
        self.model = YOLO(r'C:\Users\31637\Downloads\best.pt')  
        
    def quality_control(self, conf_threshold):
        if not self.cap.isOpened():
            print("Camera can't be opened")
        else:
            print("Camera on and picture taken")
            
            ret, frame = self.cap.read()
            
            color_correct = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            wb = cv.xphoto.createSimpleWB()
            img = wb.balanceWhite(color_correct)

            cropped_img = img[self.crop_y1:self.crop_y2, self.crop_x1:self.cropx2]
            
            results = model(cropped_img, self.conf_threshold)
            
            try:
                result_img = results[0].plot()

                for result in results[0].obb.xywhr: 
                    x_center = int(result[0])  
                    y_center = int(result[1])
                    print('Center bb:', (x_center, y_center))

                plt.imshow(result_img) 
                plt.axis('on')
                plt.show()
            except Exception as e:
                print('No camera')


camera = Vision()
camera.quality_control(0.80)