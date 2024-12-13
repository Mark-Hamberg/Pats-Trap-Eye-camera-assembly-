#!/usr/bin/env python
# coding: utf-8

# In[1]:


import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
from ultralytics import YOLO


# ### Defenitions

# In[2]:


def translations(image):
    color_correct = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    wb = cv.xphoto.createSimpleWB()
    img = wb.balanceWhite(color_correct)

    cropped_img = img[180:350, 200:500]
    
    return cropped_img


# In[3]:


def apply_yolov8(image, confidence_threshold):
    model = YOLO(r'C:\Users\31637\Downloads\best.pt')  
    results = model(image, conf=confidence_threshold)         
    return results


# In[4]:


def bounding_box(image, results):
    annotated_img = image.copy()
    result_img = results[0].plot()

    for result in results[0].obb.xywhr: 
        x_center = int(result[0])  
        y_center = int(result[1])
        
    plt.imshow(result_img) 
    plt.axis('on')
    plt.show()
    print('Center bb:', (x_center, y_center))
    
    return x_center, y_center


# ### Image

# In[5]:


cap = cv.VideoCapture(2)
if not cap.isOpened():
    print("Camera kan niet openen")
    exit()
else:
    print("Camera aan")


# In[36]:


ret, frame = cap.read()

confidence_threshold = 0.80

image = translations(frame)
results = apply_yolov8(image, confidence_threshold)
try:
    bb = bounding_box(image, results)
except: 
    UnboundLocalError
    bb = (0,0)
    print('Geen camera geplaatst')


# In[37]:


if 130 < bb[0] < 185:
    print('Camera zit goed')
else:
    print('Camera zit niet goed')


# In[ ]:





# In[ ]:




