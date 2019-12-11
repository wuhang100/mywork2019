#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Dec  7 14:10:01 2019

@author: wuhang
"""

import tensorflow as tf
import numpy as np
import sys
import os
import cv2
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'models'))
import model

# 'data/task2/0.jpg'
def import_img(img_base, img_num):
    img_dir = img_base + str(img_num) + '.jpg'
    img = cv2.imread(img_dir,0)
    return img

def show_img(img, img_name='img'):
    cv2.imshow(img_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
def show_img_group(img_group, img_name='img'):
    for i in range(np.shape(img_group)[0]):
        cv2.imshow(img_name+'+'+str(i), img_group[i])
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
def gamma_trans(img,gamma):
    gamma_table = [np.power(x/255.0,gamma)*255.0*5.0 for x in range(256)]
    gamma_table = np.round(np.array(gamma_table)).astype(np.uint8)
    return cv2.LUT(img,gamma_table)

def pre_process(img):
    if (np.mean(img)<34):
        dst = cv2.equalizeHist(img)
        sharp = dst
    else:
        dst = img
        sharp_kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])
        sharp = cv2.filter2D(dst,-1,sharp_kernel)
    #blur = cv2.GaussianBlur(sharp,(3,3),0)
    _,th3 = cv2.threshold(sharp,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    th3[:,-15:]=0
    th3[:,0:40]=0
    th3[0:4,:]=0
    th3[-6:,:]=0         
    #th3 = cv2.dilate(th3, np.ones((2, 2), np.uint8), iterations=1)
    th3 = cv2.erode(th3, np.ones((1, 2), np.uint8), iterations=1)
    return img,sharp,th3

def cal_dense(img,x,y,w,h):
    sub_fig = img[y-1:y+h+1,x-1:x+w+1]
    sub_sum = np.sum(sub_fig).astype(np.float)
    dense = sub_sum/(w*h)/255.0
    return sub_fig,dense

def seg_image(img):
    binary, contours, _ = cv2.findContours(th3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sub_index = np.zeros([1,4])
    for j in range(0,len(contours)): 
        x, y, w, h = cv2.boundingRect(contours[j])
        sub_fig,dense = cal_dense(th3, x, y, w, h)
        if ((w<7) & (h<7)):
            cv2.drawContours(th3,[contours[j]],-1,0,-1)
        elif ((w<3) | (h<10)):
            cv2.drawContours(th3,[contours[j]],-1,0,-1)
        elif (cv2.contourArea(contours[j])<=20):
            cv2.drawContours(th3,[contours[j]],-1,0,-1)
        elif ((w*h>500) & (dense>0.71)):
            cv2.drawContours(th3,[contours[j]],-1,0,-1)
        elif ((w/h>2.8)):
            cv2.drawContours(th3,[contours[j]],-1,0,-1)            
        else:
            #cv2.rectangle(th3, (x,y), (x+w,y+h), (153,153,0), 1)
            #print str(x)+' '+str(y)+' '+str(w)+' '+str(h)
            sub_index = np.vstack((sub_index,[x,y,w,h]))
        sub = sub_index[1:,:]
        sub = sub[np.lexsort(sub[:,::-1].T)]
        #print sub
    return sub

def long_img(sub_fig):
    sub_group = sub_fig
    return sub_group

def img_mat(img):
    mat = img.astype(np.float)
    return mat

def make_trainset(sub_img):
    [h,w] = np.shape(sub_img)
    img_train = np.zeros([38,22])
    x = (38-h)/2
    y = (22-w)/2
    img_train[x:x+h,y:y+w]=sub_img
    return img_train

def one_hot(num_cha):
    z_add = np.zeros([1,34])
    z_add[0,num_cha] = 1
    return z_add

if __name__ == '__main__':
    train_set = np.zeros([1,38,22])
    z_set = np.zeros([1,34])
    for i in range(0,160):
        if os.path.exists('data/task2/'+str(i)+'.jpg'):
            img = import_img('data/task2/',i)
            img = cv2.resize(img,(190,45))
            print np.shape(img)            
            img,sharp,th3 = pre_process(img)            
            sub = seg_image(th3)
            
            print '******'       
            img_group = np.array([img,sharp,th3])
            show_img_group(img_group, str(i))

            for m in range(np.shape(sub)[0]):
                [x0,y0,w0,h0] = sub[m,:]
                sub_fig,dense1 = cal_dense(th3,int(x0),int(y0),int(w0),int(h0))
                
                if ((h0>36)|(w0>20)):
                    sub_group = long_img(sub_fig)
                    #show_img_group(img_group, str(i))
                    #show_img(sub_fig, str(m))
                else:
                    show_img(sub_fig, str(m))
                    sub_train = make_trainset(sub_fig)
                    show_img(sub_train, str(m)+' train')                              
                
                    use = input("Use in training set? ")
                    if (use == 1):
                        train_add = np.reshape(sub_train,[1,38,22])
                        train_set = np.append(train_set, train_add, axis=0)
                        print np.shape(train_set)
                        num_cha = input("Input the number or character? ")
                        z_add = one_hot(num_cha)
                        z_set = np.vstack((z_set,z_add))
                        print np.shape(z_set)
                    
    train = train_set[1:,:,:]
    z = z_set[1:,:]
    np.save('train_data/train.npy',train)
    np.save('train_data/z.npy',z)
                   