import argparse
import os
import sys
import numpy as np
import h5py
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import math

parser = argparse.ArgumentParser()
parser.add_argument('--object_name', default='airplane', help='Object for resampling')
parser.add_argument('--grid_length', type=float, default=0.05, help='Grid length')
parser.add_argument('--rotate_about', default='z', help='Axis for rotate')
FLAGS = parser.parse_args()
label_name = FLAGS.object_name
delta = FLAGS.grid_length
axis = FLAGS.rotate_about

labels = ['airplane',
        'bathtub',
        'bed',
        'bench',
        'bookshelf',
        'bottle',
        'bowl',
        'car',
        'chair',
        'cone',
        'cup',
        'curtain',
        'desk',
        'door',
        'dresser',
        'flower_pot',
        'glass_box',
        'guitar',
        'keyboard',
        'lamp',
        'laptop',
        'mantel',
        'monitor',
        'night_stand',
        'person',
        'piano',
        'plant',
        'radio',
        'range_hood',
        'sink',
        'sofa',
        'stairs',
        'stool',
        'table',
        'tent',
        'toilet',
        'tv_stand',
        'vase',
        'wardrobe',
        'xbox']

def load_h5(h5_filename):
    f = h5py.File(h5_filename)
    data = f['data'][:]
    label = f['label'][:]
    return data, label

def point_distance(point_one,point_two):
    return np.linalg.norm(point_one - point_two)



def real_data_for_discriminator(data_dir, label_name):
    data, label = load_h5(data_dir)
    pointcloud = np.array([data[0]])
    for m in range (0,np.size(label,0)):
        if label[m,0] == labels.index(label_name):
            pointcloud_single = np.array([data[m]])
            pointcloud = np.vstack((pointcloud,pointcloud_single))
    return pointcloud[1:]

def pointcloud_rotate(pointcloud, theta, rotate_axis=None):
    if rotate_axis is not None:
        if rotate_axis == 'x':
            R = np.array([[1,0,0],
                          [0,math.cos(theta),math.sin(theta)],
                          [0,-math.sin(theta),math.cos(theta)]])
        if rotate_axis == 'z':
            R = np.array([[math.cos(theta),math.sin(theta),0],
                           [-math.sin(theta),math.cos(theta),0],
                           [0,0,1]])
        if rotate_axis == 'y':
            R = np.array([[math.cos(theta),0,math.sin(theta)],
                           [0,1,0],
                           [-math.sin(theta),0,math.cos(theta)]])
    else:
        R = np.array([[1,0,0],
                      [0,1,0],
                      [0,0,1]])
    pointcloud2 = np.dot(R, pointcloud.T)
    return pointcloud2


def pointcloud_downsample(cloud_input, delta, rotate_axis == None, theta=0, step=0.05):
    
    

if __name__ == "__main__":
    data_dir = '/home/wuhang/tensorwork/pointgan/data/modelnet40_ply_hdf5_2048/ply_data_test0.h5'
    # Step 1: real full 3D data for discriminator
    pointcloud = real_data_for_discriminator(data_dir, label_name) 
    # Step 2: downsample real data
    pointckoud2 = pointcloud_downsample(pointcloud, delta, axis, theta=0, step=0.05)
    # Step 3: save data
    csv_data_path =     
    if not os.path.exists(csv_data_path):
        os.makedirs(csv_data_path) 
    


pointcloud = data[1,:,:]



fig=plt.figure(dpi=120)
ax=fig.add_subplot(111,projection='3d')

theta = 1.5

pointcloud2 = pointcloud_rotate_x(pointcloud, theta)
x = pointcloud2[:,0]
y = pointcloud2[:,1]
z = pointcloud2[:,2]
ax.cla()
ax.scatter(x,y,z,c='b',marker='.',s=2,linewidth=1,alpha=1,cmap='spectral')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)
plt.show()

print 'The data size before filter is' + np.size(pointcloud2,0)

lim_low_x = -1
lim_high_x = lim_low_x + delta
lim_low_y = -1
lim_high_y = lim_low_y + delta
pointcloud2_x = pointcloud2[:,0]
pointcloud2_y = pointcloud2[:,1]
pointcloud_oneside = [0,0,0]

while (lim_high_y <= 1):
    while (lim_high_x <= 1):
        pos_x=np.where(((pointcloud2_x>=lim_low_x) & (pointcloud2_x<lim_high_x)))
        pos_y=np.where(((pointcloud2_y>=lim_low_y) & (pointcloud2_y<lim_high_y)))
        pos = [val for val in pos_x[0] if val in pos_y[0]]
        pointcloud_select = pointcloud2[pos,:]    
        if (np.shape(pointcloud_select)[0]>0):
            point_oneside = pointcloud_select[np.where(pointcloud_select == np.min(pointcloud_select[:,2]))[0][0],:]
            pointcloud_oneside = np.vstack((pointcloud_oneside,point_oneside))
        lim_low_x = lim_low_x+delta
        lim_high_x = lim_high_x+delta
    lim_low_y = lim_low_y+delta
    lim_high_y = lim_high_y+delta
    lim_low_x = -1
    lim_high_x = lim_low_x + delta

pointcloud_oneside = pointcloud_oneside[1:,:]
print 'The data size after filter is' + np.size(pointcloud_oneside,0)

fig=plt.figure(dpi=120)
ax=fig.add_subplot(111,projection='3d')

x = pointcloud_oneside[:,0]
y = pointcloud_oneside[:,1]
z = pointcloud_oneside[:,2]
ax.cla()
ax.scatter(x,y,z,c='b',marker='.',s=2,linewidth=1,alpha=1,cmap='spectral')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)
plt.show()

filename = "/home/wuhang/tensorwork/pointgan/data/"+"/side_train"+".csv"
print filename
np.savetxt(filename, pointcloud_oneside, delimiter = ',')
