import cv2
import numpy as np
import glob
import re
import os

# 1. Hhow to use
"""
frame1 = cv2.imread('0001.jpg')
frame2 = cv2.imread('0002.jpg')
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255

flow = cv2.calcOpticalFlowFarneback(prvs, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
hsv[...,0] = ang*180/np.pi/2
hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

# seperate
# Change here
horz = cv2.normalize(flow[...,0], None, 0, 255, cv2.NORM_MINMAX)     
vert = cv2.normalize(flow[...,1], None, 0, 255, cv2.NORM_MINMAX)
horz = horz.astype('uint8')
vert = vert.astype('uint8')

cv2.imwrite('flow_x.jpg',horz)
cv2.imwrite('flow_y.jpg',vert)
cv2.imwrite('flow_xy.jpg',rgb)
"""

# to sort files according to file name with numbers
pat = re.compile("(\d+)\D*$")
def key_func(x):
    mat=pat.search(os.path.split(x)[-1]) # match last group of digits
    if mat is None:
        return x
    return "{:>10}".format(mat.group(1)) # right align to 10 digits.

# to find files in all sub directories
def list_files(dir):                                                                                                  
    r = []                                                                                                            
    subdirs = [x[0] for x in os.walk(dir)]                                                                            
    for subdir in subdirs:                                                                                            
        files = os.walk(subdir).next()[2]                                                                             
        if (len(files) > 0):                                                                                          
            for file in files:                                                                                        
                r.append(subdir + "/" + file)                                                                         
    return r


# extract optical flow
def extract_flow(path, key_func):

    int_cnt = 1
    file_path = path + "/*.jpg"

    print file_path
    for cur_img in sorted(glob.glob(file_path), key=key_func):

        file_name = os.path.basename(cur_img)

        #print cur_img
        #print file_name

        base_name = os.path.splitext(file_name)[0]
        print base_name

        if int_cnt == 1:
            frame1 = cv2.imread(cur_img)
            hsv = np.zeros_like(frame1)
            hsv[...,1] = 255
            prev_name = base_name
        else:
            frame2 = cv2.imread(cur_img)

            prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
            next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

            flow = cv2.calcOpticalFlowFarneback(prvs, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            #flow = cv2.calcOpticalFlowFarneback(prvs, next, None, 0.5, 3, 100, 3, 7, 1.5, 0)
            horz = cv2.normalize(flow[...,0], None, 0, 1, cv2.NORM_MINMAX)     
            vert = cv2.normalize(flow[...,1], None, 0, 1, cv2.NORM_MINMAX)
            mag, ang = cv2.cartToPolar(horz, vert)
            hsv[...,0] = ang*180/np.pi/2
            hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
            #print hsv[...,2]
            temp = hsv[...,2].sum()
            print temp 
            rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

            horz = cv2.normalize(flow[...,0], None, 0, 255, cv2.NORM_MINMAX)     
            vert = cv2.normalize(flow[...,1], None, 0, 255, cv2.NORM_MINMAX)
            horz = horz.astype('uint8')
            vert = vert.astype('uint8')

            # save optical flow files
            flow_x = path + '/flow_x_' + prev_name + '.jpg'
            flow_y = path + '/flow_y_' + prev_name + '.jpg'
            flow_xy = path + '/flow_xy_' + prev_name + '.jpg'

            cv2.imwrite(flow_x, horz)
            cv2.imwrite(flow_y, vert)
            cv2.imwrite(flow_xy, rgb)

            # update the previous frame
            frame1 = frame2
            prev_name = base_name

        # increase cnt
        int_cnt += 1


###############################################
# Main
###############################################
target_dir = "/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/new_robot_video/"

subdirs = [x[0] for x in os.walk(target_dir)]

for subdir in subdirs:

    path, folder = os.path.split(subdir)

    #print path
    #print folder
    #if folder.isdigit():

    print subdir
    print "---------------------"
    # extract optical flow
    extract_flow(subdir, key_func)



print("done!!")

