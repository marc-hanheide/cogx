#!/usr/bin/python
## walter wohlkinger
import sys
import os
from xml.dom.minidom import parse
import string
import numpy
import cv
import shutil
import ftplib
import os
import sys
import traceback
import subprocess

def convert_pcds_TestDatabase():
    directory = '/home/walter/3DNET/Cat10_TestDatabase/pcd/'
    fileList = os.listdir(directory)
    for fn in fileList:
        if os.path.isfile(directory + fn):
            print fn
            subprocess.Popen("convert_pcd_ascii_binary " + directory + fn + " " + directory + fn + " 1", shell=True,stdout=None,stderr=None).wait()
    

def convert_pcds_Descriptors():
    directory = '/home/walter/3DNET/Cat10_ModelDatabase_trained/level_1'
    for fn1 in os.listdir(directory): # class
        if os.path.isdir(directory + '/' + fn1):
            for fn2 in os.listdir(directory + '/' + fn1): # item
                if os.path.isdir(directory + '/' + fn1 + '/' + fn2):
                    for fn3 in os.listdir(directory+ '/' + fn1 + '/' + fn2): # descriptors-names
                        if os.path.isdir(directory+ '/' + fn1 + '/' + fn2 + '/' + fn3):
                            if fn3 == 'esf':
                                for fn4 in os.listdir(directory+ '/' + fn1 + '/' + fn2 + '/' + fn3): # pcd histograms 
                                    if fn4.endswith('.pcd') and fn4.startswith('descriptor_'):
                                        print directory+ '/' + fn1 + '/' + fn2 + '/' + fn3 + '/' + fn4
                                        subprocess.Popen("convert_pcd_ascii_binary " + directory+ '/' + fn1 + '/' + fn2 + '/' + fn3 + '/' + fn4 + " " + directory+ '/' + fn1 + '/' + fn2 + '/' + fn3 + '/' + fn4 + " 0", shell=True,stdout=None,stderr=None).wait()


   
if __name__ == "__main__":   
#    convert_pcds()
    convert_pcds_Descriptors()
