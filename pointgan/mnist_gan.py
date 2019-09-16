#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 14 21:28:27 2019

@author: wuhang
"""

from __future__ import print_function
import numpy as np
import tensorflow as tf
import pickle
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import sys
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, './utils'))
sys.path.append(os.path.join(BASE_DIR, './models'))
import model

if not os.path.exists('data/'):
    os.makedirs('data/')
sys.path.append(os.path.join(BASE_DIR, './data'))

from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets('./data/')

i = 100
img = mnist.train.images[i]
print (img.shape)
plt.imshow(img.reshape((28, 28)), cmap='Greys_r')
print("Label: {}".format(mnist.train.labels[i]))

def get_loss(inputs_real, inputs_noise, smooth=0.05):
    # b x 100 -> b x 28 x 28 x 1
    g_outputs = model.generator(inputs_noise, is_training=True, reuse_mode=False)
    # b x 28 x 28 x 1 -> b x 1
    d_logits_real, d_outputs_real = model.discriminator(inputs_real, is_training=True, reuse_mode=False)
    d_logits_fake, d_outputs_fake = model.discriminator(g_outputs, is_training=True, reuse_mode=True)
    
    g_loss = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=d_logits_fake, 
                                                                    labels=tf.ones_like(d_outputs_fake)*(1-smooth)))
    d_loss_real = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=d_logits_real,
                                                                         labels=tf.ones_like(d_outputs_real)*(1-smooth)))
    d_loss_fake = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=d_logits_fake,
                                                                         labels=tf.zeros_like(d_outputs_fake)))
    d_loss = tf.add(d_loss_real, d_loss_fake)
    return g_loss, d_loss

def get_optimizer(g_loss, d_loss, learning_rate=0.001, beta1=0.4):
    train_vars = tf.trainable_variables()
    
    g_vars = [var for var in train_vars if 'generator' in var.name]
    d_vars = [var for var in train_vars if 'discriminator' in var.name]

    with tf.control_dependencies(tf.get_collection(tf.GraphKeys.UPDATE_OPS)):
        g_opt = tf.train.AdamOptimizer(learning_rate).minimize(g_loss, var_list=g_vars)
        d_opt = tf.train.AdamOptimizer(learning_rate).minimize(d_loss, var_list=d_vars)
    
    return g_opt, d_opt

def plot(samples):
    fig = plt.figure(figsize=(4, 4))
    gs = gridspec.GridSpec(4, 4)
    gs.update(wspace=0.05, hspace=0.05)

    for i, sample in enumerate(samples):
        ax = plt.subplot(gs[i])
        plt.axis('off')
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_aspect('equal')
        plt.imshow(sample.reshape(28, 28), cmap='Greys_r')

    return fig

def sample_noise(m, n):
    return np.random.uniform(-1., 1., size=[m, n])

def get_figure(inputs_noise):
    g_outputs = model.generator(inputs_noise, is_training=False, reuse_mode=True)
    return g_outputs

batch_size = 64
noise_size = 100
epochs = 5
n_samples = 25
learning_rate = 0.001
beta1 = 0.4
global_step = tf.Variable(0,trainable=False)

l_rate = tf.train.exponential_decay(0.001,global_step,mnist.train.num_examples/batch_size,0.99,staircase=True)

inputs_real = tf.placeholder(tf.float32, [batch_size, 28, 28, 1], name='inputs_real')
inputs_noise = tf.placeholder(tf.float32, [batch_size, 100], name='inputs_noise')

gen_loss, dis_loss = get_loss(inputs_real, inputs_noise, smooth=0.1)
gen_opt, dis_opt = get_optimizer(gen_loss, dis_loss, l_rate)
gen_fig = get_figure(inputs_noise)

sess = tf.Session()
sess.run(tf.global_variables_initializer())

batch_noise_sample = sample_noise(batch_size, noise_size)

epochs = 20
STEP = 0
for e in range(epochs):
    for step in range(mnist.train.num_examples//batch_size):
        print (step)    
        batch_images, _ = mnist.train.next_batch(batch_size)
        batch_images = batch_images.reshape(-1,28,28,1)

        _ = sess.run(gen_opt, feed_dict={inputs_real: batch_images, inputs_noise: sample_noise(batch_size, noise_size),
                                         global_step: STEP})
        _ = sess.run(dis_opt, feed_dict={inputs_real: batch_images, inputs_noise: sample_noise(batch_size, noise_size),
                                         global_step: STEP})
        print ("learning_rate: %f"% (sess.run(tf.train.AdamOptimizer(l_rate)._lr,feed_dict={global_step: STEP})))
        STEP = STEP+1
        
    image = sess.run(gen_fig, feed_dict={inputs_noise: sample_noise(batch_size, noise_size)})
    for i in range(6):
        image_show = image[i,:,:,0]
        plt.imshow(image_show, cmap='Greys_r')
        plt.axis('off') 
        plt.savefig('./out/image'+str(e)+str(i)+'.png')
        plt.show()
        
    print (epochs)
    
    
    