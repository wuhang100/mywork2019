#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 23:06:17 2019

@author: wuhang
"""

import tensorflow as tf
import numpy as np
import sys
import os
import cv2
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'utils'))
sys.path.append(os.path.join(BASE_DIR, 'models'))
import model

def show_img(img, img_name='img'):
    cv2.imshow(img_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if not os.path.exists('train_data/train.npy'):
    os._exit(0)
print 'Find data.'

X = np.load('train_data/train.npy')
Z = np.load('train_data/z.npy')
batch_size = 10
img_in = tf.placeholder(dtype=tf.float64, shape=[batch_size, 38, 22], name='img_input')
num_cha = tf.placeholder(dtype=tf.float64, shape=[batch_size, 34], name='img_output')

logits = model.mlp_net(img_in, False, reuse_mode = False, bn_mode = False)
out = tf.argmax(logits, 1)
    
sess = tf.Session()
sess.run(tf.global_variables_initializer())

saver = tf.train.Saver()
saver.restore(sess, tf.train.latest_checkpoint('./out/models/'))

start = 12
x_in = X[start:start+batch_size]
output = sess.run(out, feed_dict={img_in: x_in})

num_dic = {'0':'0','1':'1','2':'2','3':'3','4':'4','5':'5','6':'6','7':'7','8':'8','9':'9','10':'A','11':'B','12':'C',
           '13':'D','14':'E','15':'F','16':'G','17':'H','18':'J','19':'K','20':'L','21':'M','22':'N','23':'P','24':'Q',
           '25':'R','26':'S','27':'T','28':'U','29':'V','30':'W','31':'X','32':'Y','33':'Z'}

for i in range(batch_size):
    show_img(x_in[i].astype(np.uint8), num_dic[str(output[i])])
    