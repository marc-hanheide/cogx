#! /usr/bin/env python


# Imports
import sys
import os.path
import os
import warnings
import getopt
import time

# -----------------------------------
def exitWithHelp(error):
    print
    print 'Error: '+error
    print 
    print 'Usage: %s <model> <input_dat_file> <output_dat_file>' % (sys.argv[0])
    print
    print 'Arguments:'
    print '<model>    - Model file'
    print '<input_dat_file> - Input .dat file'
    print '<output_dat_file> - Output .dat file'
    print
    sys.exit()
# -----------------------------------


# -----------------------------------
def exitWithError(error):
    print
    print 'Error: '+error
    print
    sys.exit()
# -----------------------------------






# -----------------------------------
def main():

    # -------------------------------
    # Read the command line arguments
    # -------------------------------

    try:
        opts, args = getopt.getopt(sys.argv[1:], '')
    except Exception, (e1, e2):
        exitWithHelp(e2)
    
    if len(args) != 3:
            exitWithHelp('Incorrect arguments!')


    modelFile = args[0]
    inputDatFile = args[1]
    outputDatFile = args[2]
    
    print
    print 'Creating output dat file: '+outputDatFile
    print 'from:'
    print '=> model file: '+modelFile
    print '=> input dat file: '+inputDatFile
    
    try:
        mF = file(modelFile, 'r')
    except Exception, (e1, e2):
        exitWithError('Error opening the model file!')

    try:
        idF = file(inputDatFile, 'r')
    except Exception, (e1, e2):
        exitWithError('Error opening the input dat file!')

    try:
        odF = file(outputDatFile, 'w')
    except Exception, (e1, e2):
        exitWithError('Error opening the output dat file!')

    # Processing model
    print
    print 'Processing model file'

    # Load labels and nr_SV
    labels=[]
    nrSV=[]
    nrSVInt=[]
    nrClass=0
    line=mF.readline()
    while line[:-1]!='SV':
        if line[:8]=='nr_class':
            nrClass=int(line[:-1].split(' ')[1])
        if line[:5]=='label':
            labels=line[:-1].split(' ')[1:]
        if line[:5]=='nr_sv':
            nrSV=line[:-1].split(' ')[1:]
        line=mF.readline()

    # Convert nrSV to int
    allSVs=0
    for i in nrSV:
        nrSVInt.append(int(i))
        allSVs+=int(i)

    # Check
    if len(labels)!=nrClass or len(nrSV)!=nrClass:
        exitWithError('The model file contains errors!')
    else:
        print '=> Number of classes: '+str(nrClass)
        print '=> Labels: '+' '.join(labels)    
        print '=> Nr SVs: '+' '.join(nrSV)    
        print '=> All SVs: '+str(allSVs)
        
    def readSV():
        l=mF.readline()
        colon=l.find(':')
        if colon<0:
            return ''
        space=l.rfind(' ',0,colon)+1
        if space<0:
            return ''
        return l[space:]
        
            
    c=0    
    s=nrSVInt[0]
    for i in range(allSVs):
        svLine=readSV()            
        if svLine=='':
            exitWithError('The model file contains errors!')
        if i==s:
            c+=1
            s+=nrSVInt[c]
        odF.write(labels[c]+' '+svLine)


    # Processing input dat
    
    print 'Processing input dat file'

    line=idF.readline()    
    while line!='':
        odF.write(line)
        line=idF.readline()    
    
    mF.close()
    odF.close()
    idF.close()
    print 'Done!'
    print
    
main()


