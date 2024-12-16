#!/usr/bin/env python
# coding: utf-8

# In[20]:


import socket
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO


# In[21]:


def quality_control():
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline = rs.pipeline()
    pipeline.start(config)
    
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    
    color_correct = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    wb = cv2.xphoto.createSimpleWB()
    img = wb.balanceWhite(color_correct)
    
    cropped_img = img[200:350, 200:500]
    
    model = YOLO(r'C:\Users\31637\Downloads\best.pt')  
    results = model(cropped_img, conf=0.8, verbose = False)  

    result_img = results[0].plot()

    if results[0].obb.xywhr.numel() == 0:
        #print("No camera")
        plt.imshow(result_img) 
        plt.axis('on')
        plt.show()
        pipeline.stop()
        return 0
    else:
        for result in results[0].obb.xywhr: 
            x_center = int(result[0])  
            y_center = int(result[1])
            #print("Center:", (x_center, y_center))

            if 90 < x_center < 160:
                #print("Camera is placed correctly")
                plt.imshow(result_img) 
                plt.axis('on')
                plt.show()
                pipeline.stop()
                return 1
            else:
                #print("Camera is not placed correctly")
                plt.imshow(result_img) 
                plt.axis('on')
                plt.show()
                pipeline.stop()
                return 0   
    


# In[48]:


x = quality_control()
x


# In[41]:


HOST = '0.0.0.0'
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    server_socket, address = s.accept()
    server_socket.send(str(x).encode())


# In[ ]:





# In[ ]:




