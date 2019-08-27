#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 27 16:12:59 2019

@author: wuhang
"""

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

def batchnorm(Ylogits, is_training, bn_step):
    exp_avg = tf.train.ExponentialMovingAverage(0.998, bn_step)
    bn_epsilon = 1e-5
    Offset = tf.Variable(tf.zeros([Ylogits.get_shape()[-1].value]))
    Scale = tf.Variable(tf.ones([Ylogits.get_shape()[-1].value]))
    mean, variance = tf.nn.moments(Ylogits, [0])
    bn_avg_op = exp_avg.apply([mean, variance])
    if is_training == True:
        m = mean
        v = variance
    else:
        m = exp_avg.average(mean)
        v = exp_avg.average(variance)
    Y_bn = tf.nn.batch_normalization(Ylogits, m, v, Offset, Scale, bn_epsilon)
    return Y_bn, bn_avg_op

def fully_connection (inputs, output_num, scope_name, active_fun, avg_class,
                      reuse_mode=False, bn_mode=False, is_training=True):
    input_num = inputs.get_shape()[-1].value
    with tf.variable_scope (scope_name, reuse=reuse_mode):
        weights = get_weight (input_num,output_num,"weight")
        biases = get_bias (output_num,0.0,"bias")
    # Moving average
    if is_training == False:
        Ylogits = tf.matmul(inputs, avg_class.average(weights))+avg_class.average(biases)
    else:
        Ylogits = tf.matmul(inputs, weights)+biases  
    # Batch normalization
    if bn_mode == True:
        outputs = tf.layers.batch_normalization(Ylogits, is_training)
    else:
        outputs = Ylogits
    # Activation function
    if active_fun is not None:
        outputs = active_fun(outputs)
    return outputs

def generator(inputs, reuse, is_training, avg=None, bn_mode=False):
    G_h1 = fully_connection (inputs, 128, "gen_l1", tf.nn.relu, avg, reuse, bn_mode, is_training)
    G_log_prob = fully_connection(G_h1, 784, "gen_l2", None, avg, reuse, False, is_training)
    G_prob = tf.nn.sigmoid(G_log_prob)
    return G_prob

def discriminator(inputs, reuse, is_training, avg=None, bn_mode=False):
    D_h1 = fully_connection (inputs, 128, "dis_l1", tf.nn.relu, avg, reuse, bn_mode, is_training)
    D_logit = fully_connection(D_h1, 1, "dis_l2", None, avg, reuse, False, is_training)   
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

batch_size = 16
Z_dim = 100
X = tf.placeholder(tf.float32, shape=[batch_size, 784])
Z = tf.placeholder(tf.float32, shape=[batch_size, 100])
mnist = input_data.read_data_sets("./data/MNIST/", one_hot=True)
global_step = tf.Variable(0,trainable=False)

#generator(inputs, reuse, is_training, avg=None, bn_mode=False)
G_sample = generator(Z, False, True, None, True)
#discriminator(inputs, reuse, is_training, avg=None, bn_mode=False)
D_real, D_logit_real = discriminator(X, False, True, None, True)
D_fake, D_logit_fake = discriminator(G_sample, True, True, None, True)

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
G_sample_output = generator(Z, True, False, var_avg, True)

update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
    
with tf.control_dependencies(update_ops):
    train_D = D_solver 
    train_G = [G_solver,var_avg_op]    

sess = tf.Session()
sess.run(tf.global_variables_initializer())

if not os.path.exists('out/'):
    os.makedirs('out/')

i = 0
for step in range(200000):
    
    if step % 20000 == 0:
        samples = sess.run(G_sample_output, feed_dict={Z: sample_Z(16, Z_dim), global_step: step})
        print samples
        fig = plot(samples)
        plt.savefig('out/{}.png'.format(str(i).zfill(3)), bbox_inches='tight')
        plt.close(fig)
        i += 1
    
    X_mb, _ = mnist.train.next_batch(batch_size)

    sess.run(train_D, feed_dict={X: X_mb, Z: sample_Z(batch_size, Z_dim), global_step: step})
    sess.run(train_G, feed_dict={Z: sample_Z(batch_size, Z_dim), global_step: step})

    if step % 2000 == 0:
        print("Step: %d" % (step))
        print ("learning_rate: %f"% (sess.run(tf.train.AdamOptimizer(l_rate)._lr,feed_dict={global_step: step})))
        print("")