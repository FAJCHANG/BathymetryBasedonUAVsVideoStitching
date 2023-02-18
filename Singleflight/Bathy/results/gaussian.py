import os
import cv2
import sys
import argparse


rootpath = './downSample/'
files = os.listdir(rootpath)

filtpath = "./filt/"
kernel = 39

if not os.path.exists(filtpath):
    os.makedirs(filtpath)




for i in range(len(files)):
    frame = cv2.imread(os.path.join(rootpath, files[i]))
    h,w,c = frame.shape
    img_gaussian = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (kernel, kernel), 0)
    cv2.imwrite(filtpath + "/" + "%04d"%(i+1) + ".jpg", img_gaussian)

print("Done!")