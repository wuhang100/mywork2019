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

def mlp_net(train_data, is_training, reuse_mode = False, bn_mode = False):
    with tf.variable_scope ('mlp', reuse=reuse_mode):
        # b*38*22 -> b*836 -> b*512 -> b*512 -> b*34
        inputs = tf.reshape(train_data, [-1, 836])
        layer1 = tf_util2.fully_connection('l1',inputs,512,is_training,reuse_mode,tf.nn.relu,0.001,use_bn=bn_mode)
        layer2 = tf_util2.fully_connection('l2',layer1,512,is_training,reuse_mode,tf.nn.relu,0.001,use_bn=bn_mode)
        layer3 = tf_util2.fully_connection('l3',layer2,34,is_training,reuse_mode,None,0.001,use_bn=False)
    return layer3

def cnn_net(train_data, is_training, reuse_mode = False, bn_mode = False):
    with tf.variable_scope ('cnn', reuse=reuse_mode):
        # b*38*22 -> b*38*22*32 -> b*19*11*64 -> b*10*6*64 -> b*5*3*64
        inputs = tf.reshape(train_data, [-1, 38, 22])
        conv1 = tf_util2.conv_2d('conv1',inputs,32,is_training,reuse_mode,tf.nn.relu,'same',[2,2],(1,1),use_bn=bn_mode)
        conv2 = tf_util2.conv_2d('conv2',conv1,64,is_training,reuse_mode,tf.nn.relu,'same',[2,2],(2,2),use_bn=bn_mode)
        conv3 = tf_util2.conv_2d('conv3',conv2,64,is_training,reuse_mode,tf.nn.relu,'same',[2,2],(2,2),use_bn=bn_mode)
        conv4 = tf_util2.conv_2d('conv4',conv3,64,is_training,reuse_mode,tf.nn.relu,'same',[2,2],(2,2),use_bn=bn_mode)
        flatten = tf.reshape(conv4,[-1,5*3*64])
        # b*960 -> b*512 -> b*512 -> b*40
        layer1 = tf_util2.fully_connection('l1',flatten,512,is_training,reuse_mode,tf.nn.relu,0.001,use_bn=bn_mode)
        layer2 = tf_util2.fully_connection('l2',layer1,512,is_training,reuse_mode,tf.nn.relu,0.001,use_bn=bn_mode)
        layer3 = tf_util2.fully_connection('l3',layer2,34,is_training,reuse_mode,None,0.001,use_bn=bn_mode)
    return layer3


def pc_vae_encoder(pc_in, is_training, reuse_mode = False, bn_mode = True):
    with tf.variable_scope ('vae_encoder', reuse=reuse_mode):
        # b*2048*3*1 -> b*2048*1*64 -> b*2048*1*128 -> b*2048*1*128 -> b*2048*1*256 -> b*2048*1*256
        inputs = tf.reshape(pc_in, [-1,2048,3,1])
        layer1 = tf_util2.conv_2d ('e1',inputs,64,is_training,reuse_mode,tf.nn.relu,'valid',[1,3],(1,1),use_bn = bn_mode)
        layer2 = tf_util2.conv_2d ('e2',layer1,128,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer3 = tf_util2.conv_2d ('e3',layer2,128,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer4 = tf_util2.conv_2d ('e4',layer3,256,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer5 = tf_util2.conv_2d ('e5',layer4,256,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        # b*2048*1*256 -> b*1*1*256 -> b*256
        layer6 = tf.layers.max_pooling2d(layer5,[2048,1],[1,1],padding='valid')
        layer7 = tf.squeeze(layer6)
        # b*256 -> (z_sigma, z_mu)
        z_sigma = tf_util2.fully_connection ('es',layer7,256,is_training,reuse_mode,None,0.001,use_bn = False)
        z_mu = tf_util2.fully_connection ('em',layer7,256,is_training,reuse_mode,None,0.001,use_bn = False)
        epsion = tf.random_normal(shape=tf.shape(z_sigma),mean=0,stddev=1, dtype=tf.float32)
        latent = z_mu + tf.sqrt(tf.exp(z_sigma)) * epsion
    return latent, z_sigma, z_mu

def pc_vae_decoder(latent, is_training, reuse_mode = False, bn_mode = False, pc_cat = 1):
    with tf.variable_scope ('vae_decoder', reuse=reuse_mode):
        #latent2: b*1*256, ones*latent2: b*2048*256, points: b*2048*3 -> inputs: b*2048*259
        latent2 = tf.reshape(latent, [tf.shape(latent)[0],1,256])
        ones = tf.ones(shape=[tf.shape(latent)[0],2048,1])
        points = tf.random_normal(shape=[tf.shape(latent)[0],2048,3],mean=0,stddev=1, dtype=tf.float32, seed = pc_cat)
        inputs0 = tf.concat([points, tf.matmul(ones,latent2)], axis=2)
        # b*2048*259*1 -> b*2048*1*256 -> b*2048*1*256 -> b*2048*1*128 -> b*2048*1*128 -> b*2048*1*3 -> b*2048*3
        inputs = tf.reshape(inputs0,[-1,2048,259,1])
        layer1 = tf_util2.conv_2d ('g1',inputs,256,is_training,reuse_mode,tf.nn.relu,'valid',[1,259],(1,1),use_bn = bn_mode)
        layer2 = tf_util2.conv_2d ('g2',layer1,256,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer3 = tf_util2.conv_2d ('g3',layer2,128,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer4 = tf_util2.conv_2d ('g4',layer3,128,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer5 = tf_util2.conv_2d ('g5',layer4,3,is_training,reuse_mode,None,'valid',[1,1],(1,1),use_bn = False)
        recon = tf.squeeze(layer5)
    return recon

def pc_gan_discriminator(pc_in, is_training, reuse_mode = False, bn_mode = False):
    with tf.variable_scope ('gan_discriminator', reuse=reuse_mode):
        # b*2048*3*1 -> b*2048*1*64 -> b*2048*1*128 -> b*2048*1*256 -> b*2048*1*1024
        inputs = tf.reshape(pc_in, [-1,2048,3,1])
        layer1 = tf_util2.conv_2d ('d1',inputs,64,is_training,reuse_mode,tf.nn.relu,'valid',[1,3],(1,1),use_bn = bn_mode)
        layer2 = tf_util2.conv_2d ('d2',layer1,128,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer3 = tf_util2.conv_2d ('d3',layer2,256,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        layer4 = tf_util2.conv_2d ('d4',layer3,1024,is_training,reuse_mode,tf.nn.relu,'valid',[1,1],(1,1),use_bn = bn_mode)
        # b*2048*1*1024 -> b*1*1*1024 -> b*1024
        layer5 = tf.layers.max_pooling2d(layer4,[2048,1],[1,1],padding='valid')
        layer6 = tf.squeeze(layer5)
        # b*1024 -> b*256 -> b*256 -> b*1
        layer7 = tf_util2.fully_connection ('d7',layer6,256,is_training,reuse_mode,tf.nn.relu,0.001,use_bn = False)
        layer8 = tf_util2.fully_connection ('d8',layer7,256,is_training,reuse_mode,tf.nn.relu,0.001,use_bn = False)
        logits = tf_util2.fully_connection ('logits',layer8,1,is_training,reuse_mode,None,0.001,use_bn = False)
        outputs = tf.sigmoid(logits)
    return outputs, logits
           
