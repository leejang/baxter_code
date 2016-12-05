import os
import sys
import cv2
import numpy as np
import glob
import re
import time

sys.path.insert(0, '/home/leejang/lib/caffe/python')

import caffe
caffe.set_device(0)
caffe.set_mode_gpu()

pat = re.compile("(\d+)\D*$")

def key_func(x):
        mat=pat.search(os.path.split(x)[-1]) # match last group of digits
        if mat is None:
            return x
        return "{:>10}".format(mat.group(1)) # right align to 10 digits.

images = "/home/leejang/data/recorded_videos_on_0920_2016/scenario1/0101/*.jpg"
# save output of network
output_file = "test.txt"

# Caffe model and weights
model_def = '/home/leejang/lib/caffe/models/bvlc_alexnet/deploy.prototxt'
model_weights = '/home/leejang/lib/caffe/models/bvlc_alexnet/bvlc_reference_caffenet.caffemodel'

net = caffe.Net(model_def, model_weights, caffe.TEST)
img_mean_file = '/home/leejang/lib/caffe/python/caffe/imagenet/ilsvrc_2012_mean.npy'

# layer to extract features
layer = 'fc7'
if layer not in net.blobs:
    raise TypeError("Invalid layer name: " + layer)

# input preprocessing: 'data' is the name of the input blob == net.inputs[0]
transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_mean('data', np.load(img_mean_file).mean(1).mean(1))
transformer.set_transpose('data', (2,0,1))
transformer.set_raw_scale('data', 255.0)

#image_resize
image_resize = 227
net.blobs['data'].reshape(1,3,image_resize,image_resize)

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

    with open(output_file, 'w') as f:
        np.savetxt(f, net.blobs[layer].data[0], fmt='%.4f', delimiter='\n')

print("pre-processing done!!")
