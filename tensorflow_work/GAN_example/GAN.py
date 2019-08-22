#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 18 22:57:19 2019

@author: wuhang
"""
import tensorflow as tf
from tensorflow.examples.tutorials.mnist import input_data
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

def get_weight (input_num,output_num,name):
    size = [input_num, output_num]
    w_stddev = 1. / tf.sqrt(input_num / 2.)
    return tf.get_variable(name, size,
                           initializer=tf.truncated_normal_initializer(stddev=w_stddev))
    
def get_bias (output_num,init,name):
    return tf.get_variable(name,[output_num],initializer=tf.constant_initializer(init))

def fully_connection (inputs, output_num, scope_name, active_fun, avg_class, reuse_mode=False, bn=False, is_training=None):
    input_num = inputs.get_shape()[-1].value
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        weights = get_weight (input_num,output_num,"weight")
        biases = get_bias (output_num,0.0,"bias")
    if avg_class is not None:
        outputs = tf.matmul(inputs, avg_class.average(weights))+avg_class.average(biases)
    else:
        outputs = tf.matmul(inputs, weights)+biases
    if active_fun is not None:
        outputs = active_fun(outputs)  
    return outputs

def generator(inputs, reuse, avg=None):  
    G_h1 = fully_connection (inputs, 128, "gen_l1", tf.nn.relu, avg, reuse)
    G_log_prob = fully_connection(G_h1, 784, "gen_l2", None, avg, reuse)
    G_prob = tf.nn.sigmoid(G_log_prob)
    return G_prob

def discriminator(inputs, reuse):
    D_h1 = fully_connection (inputs, 128, "dis_l1", tf.nn.relu, None, reuse)
    D_logit = fully_connection (D_h1, 1, "dis_l2", None, None, reuse)    
    D_prob = tf.nn.sigmoid(D_logit)   
    return D_prob, D_logit

def sample_Z(m, n):
    return np.random.uniform(-1., 1., size=[m, n])

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

X = tf.placeholder(tf.float32, shape=[None, 784])
Z = tf.placeholder(tf.float32, shape=[None, 100])
batch_size = 128
Z_dim = 100
mnist = input_data.read_data_sets("./data/MNIST/", one_hot=True)
global_step = tf.Variable(0,trainable=False)

G_sample = generator(Z, False, None)
D_real, D_logit_real = discriminator(X, False)
D_fake, D_logit_fake = discriminator(G_sample, True)

theta_D = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope="dis_l1|dis_l2")
theta_G = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope="gen_l1|gen_l2")
D_loss_real = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=D_logit_real, labels=tf.ones_like(D_logit_real)))
D_loss_fake = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=D_logit_fake, labels=tf.zeros_like(D_logit_fake)))
D_loss = D_loss_real + D_loss_fake
G_loss = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=D_logit_fake, labels=tf.ones_like(D_logit_fake)))

l_rate = tf.train.exponential_decay(0.001,global_step,mnist.train.num_examples/batch_size,0.99,staircase=True)
D_solver = tf.train.AdamOptimizer(l_rate).minimize(D_loss, var_list=theta_D)
G_solver = tf.train.AdamOptimizer(l_rate).minimize(G_loss, var_list=theta_G)

var_avg = tf.train.ExponentialMovingAverage(0.99,global_step)
var_avg_op = var_avg.apply(theta_G)
G_sample_output = generator(Z, True, var_avg)

with tf.control_dependencies([G_solver,var_avg_op]):
    gen_fig = tf.no_op(name="gen_fig")

sess = tf.Session()
sess.run(tf.global_variables_initializer())

if not os.path.exists('out/'):
    os.makedirs('out/')

i = 0
for step in range(20000):
    
    if step % 2000 == 0:
        samples, samples_old= sess.run([G_sample_output,G_sample], feed_dict={Z: sample_Z(16, Z_dim), global_step: step})
        fig = plot(samples)
        plt.savefig('out/{}.png'.format(str(i).zfill(3)), bbox_inches='tight')
        plt.close(fig)
        fig = plot(samples_old)
        plt.savefig('out/{}_old.png'.format(str(i).zfill(3)), bbox_inches='tight')
        i += 1
        plt.close(fig)
    
    X_mb, _ = mnist.train.next_batch(batch_size)

    sess.run(D_solver, feed_dict={X: X_mb, Z: sample_Z(batch_size, Z_dim), global_step: step})
    sess.run(gen_fig, feed_dict={Z: sample_Z(batch_size, Z_dim), global_step: step})

    if step % 2000 == 0:
        print("Step: %d" % (step))
        print ("learning_rate: %f"% (sess.run(tf.train.AdamOptimizer(l_rate)._lr,feed_dict={global_step: step})))
        print("")