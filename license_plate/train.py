#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 18:03:59 2019

@author: wuhang
"""

import tensorflow as tf
import numpy as np
import sys
import os
import random
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'utils'))
sys.path.append(os.path.join(BASE_DIR, 'models'))

import model

def get_loss(x, z):
    logits = model.mlp_net(x, True, reuse_mode = False, bn_mode = False)
    ent_loss = tf.reduce_mean(tf.nn.sparse_softmax_cross_entropy_with_logits(logits = logits, labels = tf.argmax(z,1)))
    reg_loss = tf.add_n(tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES))
    loss = ent_loss + reg_loss
    return loss

def optimize_loss(loss, step):
    lr = tf.train.exponential_decay(0.001,step,300,0.95,staircase=True)
    with tf.control_dependencies(tf.get_collection(tf.GraphKeys.UPDATE_OPS)):
        train_step = tf.train.AdamOptimizer(learning_rate=lr).minimize(loss)
    return train_step

def validate_accuracy(x, z):
    logits = model.mlp_net(x, False, reuse_mode = True, bn_mode = False)
    accuracy = tf.reduce_mean(tf.cast(tf.equal(tf.argmax(z, 1), tf.argmax(logits, 1)),'float32'))
    return accuracy

class cluster():
    def __init__(self, img_cluster, batch_size):
        self.cluster = img_cluster
        self.shape = np.shape(img_cluster)[0]
        self.batch = batch_size
        self.id = 0
    def next_batch(self):
        #print (self.id) 
        if self.id + self.batch > self.shape:
            pc_batch = np.vstack(( self.cluster[self.id:,],
                                  self.cluster[0:self.id+self.batch-self.shape,] ))
            self.id = self.id+self.batch-self.shape
        else:     
            pc_batch = self.cluster[self.id:self.id+self.batch,]
            self.id += self.batch
        #print (np.shape(pc_batch)) 
        return pc_batch
#    def shuffle_data(pc_batch):
#        index = [i for i in range(np.shape(pc_batch)[0])] 
#        random.shuffle(index)
#        return pc_batch[index]
    
def train():
    batch_size = 20
    min_loss = 0.5
    X = np.load('train_data/train.npy')
    Z = np.load('train_data/z.npy')
    img_in = tf.placeholder(dtype=tf.float64, shape=[batch_size, 38, 22], name='img_input')
    num_cha = tf.placeholder(dtype=tf.float64, shape=[batch_size, 34], name='img_output')
    global_step = tf.Variable(0, trainable = False)
    train_data = cluster(X, batch_size)
    train_label = cluster(Z, batch_size)
    loss = get_loss(img_in,num_cha)
    opt_step = optimize_loss(loss, global_step)
    acc = validate_accuracy(img_in, num_cha)
    
    sess = tf.Session()
    sess.run(tf.global_variables_initializer())
    
    var_list = tf.trainable_variables()
    g_list = tf.global_variables()
    bn_var = [g for g in g_list if 'moving_mean' in g.name]
    bn_var += [g for g in g_list if 'moving_variance' in g.name]
    var_list += bn_var
    saver = tf.train.Saver(var_list=var_list, max_to_keep=1)
    
    print ('Pretain: VAE')
    for e in range (3000):
        loss_out, _ = sess.run([loss,opt_step],feed_dict={img_in:train_data.next_batch(),num_cha:train_label.next_batch(),global_step:e})
        if loss_out < min_loss:
            min_loss = loss_out
            saver.save(sess, './out/models/model_mlp', global_step=e)        
        if e%30 == 0:
            accurate = sess.run(acc,feed_dict={img_in:train_data.next_batch(),num_cha:train_label.next_batch()})
            print ('Tain '+str(e)+' '+ str(train_data.id) + ' loss is: '+str(loss_out)+' accuracy is: '+str(accurate*100.0)+'%')

    

if __name__ == '__main__':    
    if not os.path.exists('train_data/train.npy'):
        os._exit(0)
    if not os.path.exists('out/models/model_mlp/'):
        os.makedirs('out/models/model_mlp')
    print 'Find train data.'
    train()

    
