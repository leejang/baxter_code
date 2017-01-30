import os
import sys
import cv2
import numpy as np
import glob
import re
import time
import h5py

#sys.path.insert(0, '/home/leejang/lib/caffe/python')
sys.path.insert(0, '/home/leejang/lib/ssd_caffe/caffe/python')

import caffe
#caffe.set_device(0)
#caffe.set_mode_gpu()
caffe.set_mode_cpu()

pat = re.compile("(\d+)\D*$")

def key_func(x):
        mat=pat.search(os.path.split(x)[-1]) # match last group of digits
        if mat is None:
            return x
        return "{:>10}".format(mat.group(1)) # right align to 10 digits.

################################
# Env Setting
#

# input images
images = "/home/leejang/data/recorded_videos_on_0920_2016/scenario1/0101/*.jpg"
# - 30 is used because video is recorded with 30 FPS
# and this network is designed to predict 1 sec later (future)
num_of_images = len(glob.glob(images)) - 30

#print num_of_images

# save output of network (feature vectors)
output_file = "test.txt"
# hdf5 output file
hdf_f = h5py.File('train.h5', 'w')
# we will concatenate 10 images which has 3 chaneels with 227 * 227 image size
hdf_f.create_dataset('data', (1,3,227,227), dtype='f')
# then predict one future image which has same number of eature vectors (4096)
hdf_f.create_dataset('label', (1,4096), dtype='f')

# Caffe model and weights
model_def = '/home/leejang/lib/caffe/models/bvlc_alexnet/new_deploy.prototxt'
model_weights = '/home/leejang/lib/caffe/models/bvlc_alexnet/bvlc_reference_caffenet.caffemodel'

net = caffe.Net(model_def, model_weights, caffe.TEST)
img_mean_file = '/home/leejang/lib/caffe/python/caffe/imagenet/ilsvrc_2012_mean.npy'

# layer to extract features
layer = 'fc7'
if layer not in net.blobs:
    raise TypeError("Invalid layer name: " + layer)

################################
# Pre-processing
#

# input preprocessing: 'data' is the name of the input blob == net.inputs[0]
transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_mean('data', np.load(img_mean_file).mean(1).mean(1))
#transformer.set_mean('data', np.array([104,117,123]))
transformer.set_transpose('data', (2,0,1))
transformer.set_channel_swap('data', (2,1,0))
transformer.set_raw_scale('data', 255)

#image_resize
image_resize = 227
net.blobs['data'].reshape(1,3,image_resize,image_resize)

# processing time check
t = time.time()

image = caffe.io.load_image('/home/leejang/data/recorded_videos_on_0920_2016/scenario1/0101/frame_0.jpg')

#t = time.time()
transformed_image = transformer.preprocess('data', image)
net.blobs['data'].data[...] = transformed_image
hdf_f['data'][0] = transformed_image

# Forward pass.
output = net.forward()
feature_vectors = net.blobs[layer].data[0]/net.blobs[layer].data[0].max()
hdf_f['data'][0] = net.blobs['data'].data[0]
hdf_f['label'][0] = feature_vectors

print("get feature vector {:.3f} seconds.".format(time.time() - t))

"""
################################
# main loop
#
for test_img in sorted(glob.glob(images), key=key_func):

    file_name = os.path.basename(test_img)
    #print file_name

    # processing time check
    t = time.time()

    image = caffe.io.load_image(test_img)

    #t = time.time()
    transformed_image = transformer.preprocess('data', image)
    net.blobs['data'].data[...] = transformed_image

    # Forward pass.
    output = net.forward()

    #print("get feature vector {:.3f} seconds.".format(time.time() - t))

    # save feature vectors with simple normalization
    with open(output_file, 'w') as f:
        feature_vectors = net.blobs[layer].data[0]/net.blobs[layer].data[0].max()
        np.savetxt(f, feature_vectors, fmt='%.4f', delimiter='\n')


"""
################################
# finalizing
#

hdf_f.close()
print("pre-processing done!!")
