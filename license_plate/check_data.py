#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 11:27:48 2019

@author: wuhang
"""
import numpy as np
import sys
import os
import cv2

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'train_data'))

train = np.load('train_data/train.npy')
z = np.load('train_data/z.npy')

def show_img(img, img_name='img'):
    cv2.imshow(img_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

i = input("Input the value: ")
index = z[:,i]
n = np.where(index==1)
for m in range(np.shape(n)[1]):
    img = train[n[0][m],:,:]
    img = img.astype(np.uint8)
    show_img(img, str(i))
    print n[0][m]
