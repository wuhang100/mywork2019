#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 14 11:02:06 2019

@author: wuhang
"""

import tensorflow as tf

def fully_connection (scope_name,
                      inputs, 
                      output_num,  
                      is_training, 
                      reuse_mode, 
                      activation_fun = None,
                      use_regularizer = False,
                      use_bias = True, 
                      kernel_initializer = None,
                      use_bn = True):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        if use_regularizer == True:
            regularizer = tf.contrib.layers.l2_regularizer(0.0001, scope_name + 'l2')
        else:
            regularizer = None
        outputs = tf.layers.dense(inputs, output_num, kernel_regularizer=regularizer)
        if use_bn == True:            
            outputs = tf.layers.batch_normalization(outputs, training=is_training)
        if activation_fun is not None:
            outputs = activation_fun(outputs)
    return outputs
    
def conv_2d (scope_name,
             inputs,
             output_num,
             is_training,
             reuse_mode,
             activation_fun = None,
             padding = 'valid',
             kernel_size = [2,2],
             stride_size = (1,1),
             use_regularizer = False,
             use_bn = True):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        if use_regularizer == True:
            regularizer = tf.contrib.layers.l2_regularizer(0.0001, scope_name + 'l2')
        else:
            regularizer = None
        outputs = tf.layers.conv2d(inputs, output_num, kernel_size, stride_size, padding=padding,
                                   kernel_regularizer=regularizer)
        if use_bn == True:
            outputs = tf.layers.batch_normalization(outputs, training=is_training)
        if activation_fun is not None:
            outputs = activation_fun(outputs)
    return outputs

def conv_2d_trans (scope_name,
                   inputs,
                   output_num,
                   is_training,
                   reuse_mode,
                   activation_fun = None,
                   padding = 'valid',
                   kernel_size = [2,2],
                   stride_size = (1,1),
                   use_regularizer = False,
                   use_bn = True):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        if use_regularizer == True:
            regularizer = tf.contrib.layers.l2_regularizer(0.0001, scope_name + 'l2')
        else:
            regularizer = None
        outputs = tf.layers.conv2d_transpose(inputs, output_num, kernel_size, stride_size, padding=padding, 
                                             kernel_regularizer=regularizer)
        if use_bn == True:
            outputs = tf.layers.batch_normalization(outputs, training=is_training)
        if activation_fun is not None:
            outputs = activation_fun(outputs)
    return outputs
  