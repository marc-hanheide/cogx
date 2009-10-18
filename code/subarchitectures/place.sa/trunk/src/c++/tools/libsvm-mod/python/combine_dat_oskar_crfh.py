#! /usr/bin/env python

# Imports
import sys


# Check arguments
argc = len(sys.argv)
if (argc%2) or (argc<4):
    print "Bad arguments!"
    print "Usage: *(<input_file> <class_label>) <output_file>"  
    sys.exit(1)


outputFile = sys.argv[argc-1]
i=1
inputFiles=[]
classLabels=[]
while i<argc-1:
    inputFiles.append(sys.argv[i])
    i=i+1
    classLabels.append(sys.argv[i])
    i=i+1

# Open files
inFilePtrs=[]
for f in inputFiles:
    inFilePtrs.append(open(f, 'r'))
outFilePtr = open(outputFile, 'w')
    
# Main loop
for f in inFilePtrs:
    lines = f.readlines()
    classLabel=classLabels[inFilePtrs.index(f)]
    for l in lines:
	outFilePtr.write(classLabel+l[1:])
    
    del lines
    
    
# Close files
for f in inFilePtrs:
    f.close()    
outFilePtr.close()





