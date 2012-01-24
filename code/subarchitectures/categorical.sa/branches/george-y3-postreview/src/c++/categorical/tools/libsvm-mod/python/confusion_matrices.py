#! /usr/bin/env python
#-----------------------------------------
# confusion_matrices
# Author: Andrzej Pronobis
# Contact: pronobis@nada.kth.se
#-----------------------------------------

#---------------------------------------------
# Imports
#---------------------------------------------
import pylib
import sys


#---------------------------------------------
# Main function
#---------------------------------------------
resultFiles=["BO", "CR", "EO", "KT", "PA"]

def main():

    # Process arguments
    if len(sys.argv)==1:
	filt="train*"
    else:
        filt=sys.argv[1]

    # Get list of subdirs
    subDirs=pylib.getDirDirs(".",filt)
    
    # For each subdir
    for i in subDirs:
	print ("Analysing: %s" % i)
	
	# Get into the dir
	pylib.chDir(i)
	
	# Get subsubdirs
	subSubDirs=pylib.getDirDirs(".", "*")
	
	# For each subsubdir
	maxClassRate=0
	maxClassRateDir=""
	for j in subSubDirs:
	    # Read results
	    avgClassRate=0.0
	    for r in resultFiles:
		if not (pylib.fileExists(j+"/"+r+".result")):
		    print "Error in the results file: "+i+"/"+j+"/"+r+".result"
		    sys.exit()
		if not (pylib.fileExists(j+"/"+r+".out")):
		    print "Error in the output file: "+i+"/"+j+"/"+r+".out"
		    sys.exit()
		result=pylib.readFileContents(j+"/"+r+".result")
		result=pylib.getMatchedSubStr(result, r'Accuracy = (?P<str>[0-9\.]*)% \(', "str")
		if (result==-1):
		    print "Error in the results file: "+i+"/"+j+"/"+r+".result"
		    sys.exit()
		avgClassRate=avgClassRate+float(result)
	    avgClassRate=avgClassRate/5.0
	    # Is it max?
	    if avgClassRate>maxClassRate:
		maxClassRate=avgClassRate
		maxClassRateDir=j

	# Print max
	print "  Max class rate: %f" % maxClassRate
	print "  Max class rate gamma: " + pylib.getMatchedSubStr(maxClassRateDir, r'_g(?P<str>[0-9\.]*)_N', "str")
	
	# Creating confusion matrix for this dir
	confMatrix=[]
	for r in resultFiles:
	    # Create empty counts table
	    counts=[]
	    for rr in resultFiles:
		counts.append(0)
	    countsSum=0
	    # Read out file
	    outFile=pylib.readFileLines(maxClassRateDir+"/"+r+".out")
	    # Count results
	    minLabel=1
	    maxLabel=len(resultFiles)
	    for o in outFile:
		try:
		    tmp=int(o)
		except:
		    tmp=0
		if (tmp>=minLabel) and (tmp<=maxLabel):
		    counts[tmp-1]=counts[tmp-1]+1
		    countsSum=countsSum+1
	    for c in range(len(counts)):
		counts[c]=float(counts[c])/float(countsSum)
	    # Append to confusion matrix
	    confMatrix.append(counts)
    
	# Leave the dir
	pylib.chDir("..")

	# Write the confusion matrix to a file in a matlab format
	mMat=i.replace("-","_")
	mMat=mMat+"=["
	for r in confMatrix:
	    for c in r:
		mMat=mMat+str(c)+" ";
	    mMat=mMat+"; "
	mMat=mMat+"];\n"
	pylib.appendToFile("conf_matrices.m", mMat)

	# New line
	print "  Done!"




main()




