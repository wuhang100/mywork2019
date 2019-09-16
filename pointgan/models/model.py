#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 14 21:28:27 2019

@author: wuhang
"""

import tensorflow as tf
import numpy as np
import sys
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, '../utils'))
import tf_util2

def generator(inputs, is_training, reuse_mode):
    with tf.variable_scope ('generator', reuse=reuse_mode):
        # b x 100 -> b x 4 x 4 x 512
        layer1 = tf_util2.fully_connection ('l1', inputs, 4*4*512, is_training, reuse_mode, use_bn = False)
        layer1 = tf.reshape(layer1, [-1, 4, 4, 512])
        with tf.variable_scope ('l1', reuse=reuse_mode):
            layer1 = tf.layers.batch_normalization(layer1, training=is_training)
        layer1 = tf.nn.leaky_relu(layer1)
        
        # b x 4 x 4 x 512 -> b x 7 x 7 x 256
        layer2 = tf_util2.conv_2d_trans ('l2', layer1, 256, is_training, reuse_mode, 
                                         activation_fun = tf.nn.leaky_relu,
                                         padding = 'valid',
                                         kernel_size = [4,4],
                                         stride_size = (1,1),
                                         use_bn = True)
        
        # b x 7 x 7 x 256 -> b x 14 x 14 x 128
        layer3 = tf_util2.conv_2d_trans ('l3', layer2, 128, is_training, reuse_mode, 
                                         activation_fun = tf.nn.leaky_relu,
                                         padding = 'same',
                                         kernel_size = [3,3],
                                         stride_size = (2,2),
                                         use_bn = True)
        
        # b x 14 x 14 x 256 -> b x 28 x 28 x 1
        logits = tf_util2.conv_2d_trans ('l4', layer3, 1, is_training, reuse_mode, 
                                         padding = 'same',
                                         kernel_size = [3,3],
                                         stride_size = (2,2),
                                         use_bn = True)
        outputs = tf.tanh(logits)
    return outputs

def discriminator(inputs, is_training, reuse_mode):
    with tf.variable_scope ('discriminator', reuse=reuse_mode):
        # b x 28 x 28 x 1 -> b x 14 x 14 x 128
        layer1 = tf_util2.conv_2d ('l1', inputs, 128, is_training, reuse_mode,
                                   activation_fun = tf.nn.leaky_relu,
                                   padding = 'same',
                                   kernel_size = [3,3],
                                   stride_size = (2,2),
                                   use_bn = False)
        
        # b x 14 x 14 x 128 -> b x 7 x 7 x 256
        layer2 = tf_util2.conv_2d ('l2', layer1, 256, is_training, reuse_mode,
                                   activation_fun = tf.nn.leaky_relu,
                                   padding = 'same',
                                   kernel_size = [3,3],
                                   stride_size = (2,2),
                                   use_bn = True)
        
        # b x 7 x 7 x 256 -> b x 4 x 4 x 512
        layer3 = tf_util2.conv_2d ('l3', layer2, 512, is_training, reuse_mode,
                                   activation_fun = tf.nn.leaky_relu,
                                   padding = 'same',
                                   kernel_size = [3,3],
                                   stride_size = (2,2),
                                   use_bn = True)

        # b x 4 x 4 x 512 -> b x 4*4*512
        flatten = tf.reshape(layer3, (-1, 4*4*512))
        # b x 4*4*512 x 1 -> b x 1
        logits = tf_util2.fully_connection ('flatten', flatten, 1, is_training, reuse_mode, use_bn = False)        
        outputs = tf.sigmoid(logits)
    return logits, outputs
    