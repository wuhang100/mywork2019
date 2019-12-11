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
                      regularizer_weight = 0.001,
                      use_bias = True, 
                      weight_initializer = 'xaiver',
                      use_bn = True,
                      bn_momentum = 0.9):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        regularizer = tf.contrib.layers.l2_regularizer(regularizer_weight)
        
        if weight_initializer is None:
            initializer = None
        else:
            initializer = tf.contrib.layers.xavier_initializer(uniform=False)
            
        outputs = tf.layers.dense(inputs, output_num,
                                  kernel_initializer=initializer,
                                  kernel_regularizer=regularizer)
        if use_bn == True:            
            outputs = tf.layers.batch_normalization(outputs, momentum=bn_momentum, training=is_training)
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
             regularizer_weight = 0.001,
             weight_initializer = 'xaiver',
             use_bn = True,
             bn_momentum = 0.9):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        regularizer = tf.contrib.layers.l2_regularizer(regularizer_weight)
        
        if weight_initializer is None:
            initializer = None
        else:
            initializer = tf.contrib.layers.xavier_initializer(uniform=False)
        
        outputs = tf.layers.conv2d(inputs, output_num, kernel_size, stride_size, padding=padding,
                                   kernel_initializer=initializer,
                                   kernel_regularizer=regularizer)
        if use_bn == True:
            outputs = tf.layers.batch_normalization(outputs, momentum=bn_momentum, training=is_training)
        if activation_fun is not None:
            outputs = activation_fun(outputs)
    return outputs

def conv_3d (scope_name,
             inputs,
             output_num,
             is_training,
             reuse_mode,
             activation_fun = None,
             padding = 'valid',
             kernel_size = [2,2,2],
             stride_size = (1,1,1),
             regularizer_weight = 0.001,
             weight_initializer = 'xaiver',
             use_bn = True,
             bn_momentum = 0.9):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        regularizer = tf.contrib.layers.l2_regularizer(regularizer_weight)
        
        if weight_initializer is None:
            initializer = None
        else:
            initializer = tf.contrib.layers.xavier_initializer(uniform=False)

        outputs = tf.layers.conv3d(inputs, output_num, kernel_size, stride_size, padding=padding,
                                   kernel_initializer=initializer,
                                   kernel_regularizer=regularizer)
        if use_bn == True:
            outputs = tf.layers.batch_normalization(outputs, momentum=bn_momentum, training=is_training)
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
                   regularizer_weight = 0.001,
                   weight_initializer = 'xaiver',
                   use_bn = True,
                   bn_momentum = 0.9):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        regularizer = tf.contrib.layers.l2_regularizer(regularizer_weight)
        
        if weight_initializer is None:
            initializer = None
        else:
            initializer = tf.contrib.layers.xavier_initializer(uniform=False)
            
        outputs = tf.layers.conv2d_transpose(inputs, output_num, kernel_size, stride_size, padding=padding,
                                             kernel_initializer=initializer,
                                             kernel_regularizer=regularizer)
        if use_bn == True:
            outputs = tf.layers.batch_normalization(outputs, momentum=bn_momentum, training=is_training)
        if activation_fun is not None:
            outputs = activation_fun(outputs)
    return outputs


def conv_3d_trans (scope_name,
                   inputs,
                   output_num,
                   is_training,
                   reuse_mode,
                   activation_fun = None,
                   padding = 'valid',
                   kernel_size = [2,2,2],
                   stride_size = (1,1,1),
                   regularizer_weight = 0.001,
                   weight_initializer = 'xaiver',
                   use_bn = True,
                   bn_momentum = 0.9):
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        regularizer = tf.contrib.layers.l2_regularizer(regularizer_weight)
        
        if weight_initializer is None:
            initializer = None
        else:
            initializer = tf.contrib.layers.xavier_initializer(uniform=False)
            
        outputs = tf.layers.conv3d_transpose(inputs, output_num, kernel_size, stride_size, padding=padding,
                                             kernel_initializer=initializer,
                                             kernel_regularizer=regularizer)
        if use_bn == True:
            outputs = tf.layers.batch_normalization(outputs, momentum=bn_momentum, training=is_training)
        if activation_fun is not None:
            outputs = activation_fun(outputs)
    return outputs

