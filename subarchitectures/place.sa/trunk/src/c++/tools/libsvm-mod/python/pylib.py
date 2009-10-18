#-----------------------------------------
# pylib 0.5
# Author: Andrzej Pronobis
# Contact: pronobis@nada.kth.se
#-----------------------------------------

import glob
import os
#import sre
import re
import subprocess
import sys
import shutil
import math
import string


#---------------------------------------------
# Files and Directories
#---------------------------------------------

def getDirContents(path=".",mask="*"):
    list=glob.glob(os.path.join(os.path.expanduser(path),mask))
    list2=[]
    for l in list:
        list2.append(os.path.basename(l))
    list2.sort()
    return list2


def getDirDirs(path=".", mask="*"):
    list=glob.glob(os.path.join(os.path.expanduser(path),mask))
    list2=[]
    for l in list:
        if os.path.isdir(l):
            list2.append(os.path.basename(l))
    list2.sort()
    return list2


def getDirFiles(path=".", mask="*"):
    list=glob.glob(os.path.join(os.path.expanduser(path),mask))
    list2=[]
    for l in list:
        if os.path.isfile(l):
            list2.append(os.path.basename(l))
    list2.sort()
    return list2


def chDir(path):
    os.chdir(os.path.abspath(os.path.expanduser(path)))


def getCurDir():
    return os.getcwd()


def createFile(filePath):
    f=open(filePath, 'w')    
    f.close()


def appendToFile(filePath, string):
    f=open(filePath, 'a+')
    f.write(string)
    f.close()

def fileExists(filePath):
    return os.path.isfile(filePath)
    
    
def readFileContents(filePath):
    f=open(filePath, 'r')
    lines=f.read()
    f.close()
    return lines


def readFileLines(filePath):
    f=open(filePath, 'r')
    lines=getLines(f.read())
    f.close()
    return lines


def writeFileLines(filePath, lines):
    f=open(filePath, 'w')
    f.write(joinLines(lines))
    f.close()


#---------------------------------------------
# Running commands
#---------------------------------------------

def getCmdOutput(cmd, args):
    output = subprocess.Popen([cmd, args], stdout=subprocess.PIPE).communicate()[0]
    return output

def getCmdOutput(cmd):
    output = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
    return output


#---------------------------------------------
# Strings
#---------------------------------------------


# Returns a substring that matches the pattern. If the groupName is given, 
# only the named group can be returned, e.g.
# pylib.getMatchedSubStr("_N100_M", r'_N(?P<str>\d*)_M', "str")
# will return 100. Works for a single string or a list of strings.
# The occurNo parameter can be used to specify which occurence are we 
# looking for. If no substring matches the pattern returns -1.
# For syntax look at: http://docs.python.org/lib/re-syntax.html
def getMatchedSubStr(strings, pattern, groupName="", occurNo=1):
    # Init
    if groupName=="":
        groupName=0
    pattern=str(pattern)
    if occurNo<1: 
        occurNo=1
    occ=0
    
    # Search
    if type(strings)==list:
        for i in strings:
            start=0
            pos=0
            while (pos>=0) and (start<len(i)):
                m=re.search(pattern, str(i)[start:])
                if m!=None:
                    pos=m.start()+start
                    start=pos+len(m.group(0))
                else:
                    pos=-1

                if (pos>=0):
                    occ=occ+1
                    if occ==occurNo:
                        return m.group(groupName)
        return -1
    else:
        start=0
        pos=0
        strings=str(strings)
        while (pos>=0) and (start<len(strings)):
            m=re.search(pattern, strings[start:])
            if m!=None:
                pos=m.start()+start
                start=pos+len(m.group(0))
            else:
                pos=-1

            if (pos>=0):
                occ=occ+1
                if occ==occurNo:
                    return m.group(groupName)
        return -1



# If strings is a string, returns the position of substr in 
# the string or -1 if substr not found. If strings is a list
# of strings, returns a tuple (string no., position) or -1 if 
# not found. The occurNo parameter can be used to specify
# which occurence are we looking for.
def findSubStr(strings, substr, occurNo=1):
    occ=0
    substr=str(substr)
    if occurNo<1: 
        occurNo=1

    if type(strings)==list:
        lineNo=0
        for i in strings:
            start=0
            pos=0
            while (pos>=0):
                pos=str(i).find(substr, start)
                start=pos+len(substr)
                if (pos>=0):
                    occ=occ+1
                    if occ==occurNo:
                        return (lineNo, pos)
            lineNo=lineNo+1
        return -1
    else:
        start=0
        pos=0
        strings=str(strings)
        while (pos>=0):
            pos=strings.find(substr, start)
            start=pos+len(substr)
            if (pos>=0):
                occ=occ+1
                if occ==occurNo:
                    return pos
        return -1



# Gets list of lines stored in the string
def getLines(string):
    return string.split('\n')


# Joins a list of lines into a string 
def joinLines(lines):
    return string.join(lines,'\n')


#---------------------------------------------
# BASH-like commands
#---------------------------------------------
def touch(filePath):
    createFile(filePath)

def ls(path=".",mask="*"):
    return getDirContents(path,mask)

def mkdir(path, mode=0777):
    os.makedirs(path, mode)

def mv(src, dest):
    shutil.move(src,dest)

def cp(src, dest):
    shutil.copy(src, dest)

def pwd():
    return getCurDir()


#---------------------------------------------
# Command-line interface
#---------------------------------------------





#---------------------------------------------
# Other
#---------------------------------------------

def printNoNewLine(str):
    sys.stdout.write(str)


def minDict(dictionary):
    me=0
    for i in dictionary:
        me=dictionary[i]
        break
    for i in dictionary:
        if dictionary[i]<me:
            me=dictionary[i]
    return me


def maxDict(dictionary):
    me=0
    for i in dictionary:
        me=dictionary[i]
        break
    for i in dictionary:
        if dictionary[i]>me:
            me=dictionary[i]
    return me


#---------------------------------------------
# Matlab
#---------------------------------------------

# Writes a vale into a matlab file
def writeMatlabValue(f, name, value):
    f.write('%s=%.10f;\n'%(name, value))


# Writes a vector into a matlab file
def writeMatlabVector(f, name, table):
    f.write(name+'=[')
    for t in table:
        f.write("%.10f "%(t))
    f.write('];\n')


# Writes a matrix into a matlab file
# The matrix is defined as follows:
# [[(r1,c1)...(r1,cN)] ... [(rM,c1)...(rM,cN)]]
def writeMatlabMatrix(f, name, table):
    f.write(name+'=[')
    for r in table:
        for c in r:
            f.write("%.10f "%(c))
        f.write(';')
    f.write('];\n')


# Creates a vector of zeros
def zeros(n):
    tab=[]
    for i in range(n):
        tab.append(0)
    return tab


# Creates a matrix [n][m] of zeros
def zerosMatrix(n,m):
    mat=[]
    for i in range(n):
	tab=[]
	for j in range(m):
    	    tab.append(0)
	mat.append(tab)
    return mat


# Creates a vector of ones
def ones(n):
    tab=[]
    for i in range(n):
        tab.append(1)
    return tab


# Adds two vectors
def addVectors(v1, v2):
    if len(v1)!=len(v2):
        print "ERROR: Vectors must be of the same size!"
        raise Exception()
    tab=[]
    for i in range(len(v1)):
        tab.append(v1[i]+v2[i])
    return tab


# Adds two matrices
def addMatrices(m1, m2):
    if (len(m1)!=len(m2)):
        print "ERROR: Matrices must be of the same size!"
        raise Exception()
    for i in range(len(m1)):
	if (len(m1[i])!=len(m2[i])) or (len(m1[i])!=len(m1[0])):
    	    print "ERROR: Matrices must be of the same size!"
    	    raise Exception()	    
    n=len(m1)
    m=len(m1[0])
    mat=[]
    for i in range(n):
	tab=[]
	for j in range(m):
    	    tab.append(m1[i][j]+m2[i][j])
	mat.append(tab)
    return mat


# Substracts two vectors
def substractVectors(v1, v2):
    if len(v1)!=len(v2):
        print "ERROR: Vectors must be of the same size!"
        raise Exception()
    tab=[]
    for i in range(len(v1)):
        tab.append(v1[i]-v2[i])
    return tab


# Substracts two matrices
def substractMatrices(m1, m2):
    if (len(m1)!=len(m2)):
        print "ERROR: Matrices must be of the same size!"
        raise Exception()
    for i in range(len(m1)):
	if (len(m1[i])!=len(m2[i])) or (len(m1[i])!=len(m1[0])):
    	    print "ERROR: Matrices must be of the same size!"
    	    raise Exception()	    
    n=len(m1)
    m=len(m1[0])
    mat=[]
    for i in range(n):
	tab=[]
	for j in range(m):
    	    tab.append(m1[i][j]-m2[i][j])
	mat.append(tab)
    return mat



# Multiplies two vectors
def multiplyVectors(v1, v2):
    if len(v1)!=len(v2):
        print "ERROR: Vectors must be of the same size!"
        raise Exception()
    tab=[]
    for i in range(len(v1)):
        tab.append(v1[i]*v2[i])
    return tab


# Multiplies two matrices
def multiplyMatrices(m1, m2):
    if (len(m1)!=len(m2)):
        print "ERROR: Matrices must be of the same size!"
        raise Exception()
    for i in range(len(m1)):
	if (len(m1[i])!=len(m2[i])) or (len(m1[i])!=len(m1[0])):
    	    print "ERROR: Matrices must be of the same size!"
    	    raise Exception()	    
    n=len(m1)
    m=len(m1[0])
    mat=[]
    for i in range(n):
	tab=[]
	for j in range(m):
    	    tab.append(m1[i][j]*m2[i][j])
	mat.append(tab)
    return mat


# Divides two vectors
def divideVectors(v1, v2):
    if len(v1)!=len(v2):
        print "ERROR: Vectors must be of the same size!"
        raise Exception()
    tab=[]
    for i in range(len(v1)):
        tab.append(v1[i]/v2[i])
    return tab



# Divides a vector by a constant
def divideVector(v, c):
    tab=[]
    for i in range(len(v)):
        tab.append(v[i]/c)
    return tab

# Divides a matrix by a constant
def divideMatrix(m1,c):
    for i in range(len(m1)):
	if (len(m1[i])!=len(m1[0])):
    	    print "ERROR: Incorrect matrix!"
    	    raise Exception()	    
    n=len(m1)
    m=len(m1[0])
    mat=[]
    for i in range(n):
	tab=[]
	for j in range(m):
    	    tab.append(m1[i][j]/c)
	mat.append(tab)
    return mat


# Get sqrt of vector elements
def sqrtVector(v):
    tab=[]
    for i in range(len(v)):
        tab.append(math.sqrt(v[i]))
    return tab


# Get sqrt of matrix elements
def sqrtMatrix(m1):
    for i in range(len(m1)):
	if (len(m1[i])!=len(m1[0])):
    	    print "ERROR: Incorrect matrix!"
    	    raise Exception()	    
    n=len(m1)
    m=len(m1[0])
    mat=[]
    for i in range(n):
	tab=[]
	for j in range(m):
    	    tab.append(math.sqrt(m1[i][j]))
	mat.append(tab)
    return mat


