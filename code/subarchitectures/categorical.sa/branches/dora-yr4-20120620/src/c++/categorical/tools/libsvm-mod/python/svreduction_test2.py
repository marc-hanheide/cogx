#! /usr/bin/env python
# +---------------------+------------------------------+
# | SVReduction Test    | Author: Andrzej Pronobis     |
# | Version 0.2         | E-mail: pronobis@nada.kth.se |
# |			| Modify: Roger		       | 
# +---------------------+------------------------------+
# | This script was designed to simplify the threshold |
# | adjustment process. The script runs the svm-reduce |
# | executable to reduce the given model and then runs |
# | svm-predict to obrain the classification rate.     |
# | This operation may be performed once for a given   |
# | value of threshold or may be repeated multiple     |
# | times in order to find the dependency between the  |
# | reduction rate and classification rate.            |
# | The threshold is then altered automatically by the |
# | script.                                            |
# |                                                    |
# | The results produced by the script can be stored   |
# | in a result file.                                  |
# |                                                    |
# | The usage is described in the command line help.   |
# | Path to the directory containing the libSVM        |
# | executables is stored in the libSVMDir variable.   |
# | 						       |
# | Modify:					       |
# | The modify version is not as optimial as the old   |
# | one, but much faster with similar result	       |
# +----------------------------------------------------+

# ------------------------------------------------------
# Parameters
# ------------------------------------------------------

# Path to the directory containing the libSVM executables. 
# The path can be either absolute or relative to the 
# directory of the script.
paramLibSVMDir = '.' 


# Parameters of the automatic threshold selection algorithm


# Coarse threshold value.
thrList = [0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6]

# Value indicating how the reduction rate should change 
# in consecutive experiments. If the difference between
# the reduction rate in current and previous experiment will
# fall below minOverallRRDelta, the threshold step will be
# increased. Similarly, if the difference will be greater
# than maxOverallRRDelta, the threshold will be decreased.
minOverallRRDelta = 0.01 # n2r1
maxOverallRRDelta = 0.1 # n2r1

# This value indicates how many times in a row the the threshold
# step may be decremented. Such a contraint prevents infinite
# decrementing.
maxThresholdDecrementsStep= 15

# -------------------------------------------------------

# Imports
import sys
import os.path
import os
import warnings
import getopt
import time


# Prints command line help and exits
def exitWithHelp(error):
    print
    print 'Error: '+error
    print
    print 'Usage: %s [<options>] <training_set> <input_model> <output_model> <test_set> <n> <r>' % (sys.argv[0])
    print
    print 'Options:'
    print '-t <threshold>      -> Manual threshold adjustment'
    print '-d <min_class_rate> -> Find the dependency between the reduction rate and classification rate.'
    print '                       The threshold will be increased automaticaly until the classification'
    print '                       rate will fall below the <min_clas_rate> percent of the initial class.'
    print '                       rate.'
    print '-s <result_file>    -> Save the results to <result_file> '                      
    print
    print 'Arguments:'
    print '<training_set> -> The .dat file containing training data'
    print '<input_model>  -> The model to be reduced'
    print '<output_model> -> The reduced model'
    print '<test_set>     -> The .dat file conaining test data'
    print '<n> <r>        -> The svm-reduce parameters'
    print
    sys.exit()


def exitWithError(error):
    print
    print 'Error: '+error
    print
    sys.exit()

# Finds a string containing the substring in the given string list
def findInList(strList, substr):
    for s in strList:
        if s.find(substr) >= 0:
            return s
    return ''


# Finds a result obtained for the given threshold in the given list
def findThresholdInList(thrList, threshold):
    for t in thrList:
        if str(t[0])==str(threshold):
            return t
    return (-1, -1, -1, -1, -1, -1)


# Runs the svm-reduce executable.
# Returns the reduction rates.
def startReduction(libSVMPath, trainingSet, inputModel, outputModel, n, r, threshold):
    # Extracts the reduction rate from the given string
    def readReductionRate(line):
        start = line.find('Reduced: ')
        start += 9
        end = line.find('%', start+1)
        return line[start:end]
    def readSVs(line):
        start = line.find('After: ')
        start += 7
        end = line.find(' SVs', start+1)
        return line[start:end]

    # Prepare arguments
    svmReduceExec = os.path.join(libSVMPath, 'svm-reduce')
    svmReduceArgs = "-a 1 -n %s -r %s -t %s %s %s %s" % (n, r, threshold, trainingSet, inputModel, outputModel)
    # Execute binary
    lines = os.popen(svmReduceExec+' '+svmReduceArgs+' 2>&1').readlines()

    if len(findInList(lines, 'Reading model')) == 0:
	# DEBUG INFO
        print lines        
        exitWithError('Couldn\'t execute svm-reduce!')
    # Process results
    overallLine = findInList(lines, '* Overall')
    summedupLine = findInList(lines, '* Summed up')
    
    if len(summedupLine) == 0:
        # DEBUG INFO
        print lines        
        exitWithError('Error occured during reduction!')

    return (float(readReductionRate(overallLine)), int(readSVs(overallLine)), \
            float(readReductionRate(summedupLine)), int(readSVs(summedupLine)))




# Runs the svm-predict executable.
# Returns the classification rate.
def startPrediction(libSVMPath, model, testSet):
    # Prepare arguments
    svmPredictExec = os.path.join(libSVMPath, 'svm-predict')
    tmpOutFile = os.tmpnam()
    svmPredictArgs = "%s %s %s" % (testSet, model, tmpOutFile)
    # Execute binary
    lines = os.popen(svmPredictExec+' '+svmPredictArgs+' 2>&1').readlines()
    try:
        os.remove(tmpOutFile)
    except:
        a=5
    # Process results
    accuracyLine = findInList(lines, 'Accuracy')
    if len(accuracyLine) == 0:
        exitWithError('Error occured while predicting!')
    start = accuracyLine.find('Accuracy');
    start += 11
    end = accuracyLine.find('%', start+1);
    return float(accuracyLine[start:end])

# Appends one line containing results to the result file (if the S option is turned on)
def saveResult(optS, resultFile, result):
    f = file(resultFile, 'a+')
    f.write(str(result[0])+' '+str(result[1])+' '+str(result[2])+' '+str(result[3])+' '+str(result[4])+' '+str(result[5])+'\n')
    f.close()


def printResult(result):
    print '-> [Threshold: %s][Overall RR/SVs: %s%%/%s][Summed Up RR/SVs: %s%%/%s][Class. Rate: %s%%]' \
    % (str(result[0]), str(result[1]), str(result[2]), str(result[3]), str(result[4]), str(result[5]))
    sys.stdout.flush()

    
    
# Main function
def main():

    # --------------
    # Initialization
    # --------------

    # Read the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:], 't:d:s:')
    except Exception, (e1, e2):
        exitWithHelp(e1)
    
    if len(args) != 6:
        exitWithHelp('Incorrect number of command line arguments!')
    
    argTrainingSet = args[0]
    argInputModel = args[1]
    argOutputModel = args[2]
    argTestSet = args[3]
    argN = args[4]
    argR = args[5]

    optD=False
    optT=False
    optS=False
    for o in opts:
        if o[0] == '-d':
            optD = True
            argMinClassRate = o[1]
        if o[0] == '-t':
            optT = True
            argThreshold = o[1]
        if o[0] == '-s':
            optS = True
            argResultFile = o[1]

    # Check options
    if optD and optT:
	exitWithHelp('The -d and -t options cannot be used in the same time!')
    if (not optD) and (not optT):
	exitWithHelp('Either the -d or -t option must be used!')
        

    # Check numeric params
    try:
	argNInt = int(argN)
	if argNInt<1 or argNInt>2:
	    raise Exception()
    except:
	exitWithHelp('Incorrect value of the n parameter: '+argN)
	
    try:
	argRInt = int(argR)
	if argRInt<1 or argRInt>3:
	    raise Exception()
    except:
	exitWithHelp('Incorrect value of the r parameter: '+argR)

    if optT:
        try:
    	    argThresholdFloat = float(argThreshold)
    	    if argThresholdFloat<0:
    	        raise Exception()
        except:
	    exitWithHelp('Incorrect value of the threshold parameter: '+argThreshold)

    if optD:
        try:
	    argMinClassRateFloat = float(argMinClassRate)
	    if argMinClassRateFloat<0 or argMinClassRateFloat>100:
	        raise Exception()
        except:
	    exitWithHelp('Incorrect value of the <min_class_rate> parameter: '+argMinClassRate)
    
    # Check files
    if not os.path.isfile(argTrainingSet):
	exitWithHelp('Incorrect path to the training set: '+argTrainingSet)
	
    if not os.path.isfile(argInputModel):
	exitWithHelp('Incorrect path to the input model: '+argInputModel)

    if not os.path.isfile(argTestSet):
	exitWithHelp('Incorrect path to the test set: '+argTestSet)

    # Find absolute path to the libsvm
    scriptPath = os.path.abspath(sys.path[0])
    curPath = os.getcwd()
    os.chdir(scriptPath)
    libSVMPath = os.path.abspath(paramLibSVMDir)
    os.chdir(curPath)

    # Turn off the warning
    warnings.filterwarnings('ignore', 'tmpnam is a potential security risk to your program')

    # -----------------------    
    # MAIN PART OF THE SCRIPT
    # -----------------------

    # Runs startReduction and startPrediction
    def reduceAndPredict(threshold):
        # Run reduction
        reductionResult=startReduction(libSVMPath, argTrainingSet, argInputModel, argOutputModel, argN, argR, str(threshold))
        # Run prediction
        predictionResult=startPrediction(libSVMPath, argOutputModel, argTestSet) 
        # Returns: (threshold, overall reduction rate, summedup reduction rate, classification rate)
        return (threshold, reductionResult[0], reductionResult[1], reductionResult[2], reductionResult[3], predictionResult)
    
    def convertLine(line):
	    data[0] = float(line[0])
	    data[1] = float(line[1])
	    data[2] = int  (line[2])
	    data[3] = float(line[3])
	    data[4] = int  (line[4])
	    data[5] = float(line[5])
	    return data
    
    def getMaxPos(list,elemnr):
	    max = list[0]
	    for i in list:
		    if i[elemnr]>max[elemnr]:
			    max = i
	    return max
 
    decrementsStep = 0 
    startTime = time.time()

    # Manual threshold selection
    if optT:
        # Returns: (threshold, overall reduction rate, summedup reduction rate, classification rate)
        result = reduceAndPredict(argThresholdFloat)
        if optS:
            saveResult(optS, argResultFile, result)

        totalTime = time.localtime(time.time()-startTime)

        # Print the results
        print
        print 'Threshold: '+str(result[0])
        print 'Overall reduction rate/number of SVs: '+str(result[1])+'/'+str(result[2])
        print 'Summed up reduction rate/number of SVs: '+str(result[3])+'/'+str(result[4])
        print 'Classification rate: '+str(result[5])
        print 'Total time: %d:%d:%d' % (totalTime[3]-1,totalTime[4],totalTime[5]) 
        print
        sys.exit()


    # Find dependency betwen the classification rate and reduction rate
    if optD:

        resultList=[]
	
	#Find the initial situation
	print
	print 'Checking for the initial classification rate and number of support vectors:'
	sys.stdout.flush()
	initResult = reduceAndPredict(1e-20)
	resultList.append(initResult)
	printResult(initResult)
	if optS:
		saveResult(optS,argResultFile,initResult)
	
	minClassRate = initResult[5]*(float(argMinClassRate)/100)
	
	#Coarse Classification Rate
	print
	print 'Checking the dependancy between classification rate and threshold value'
	
	
	index = int(100 - round(argMinClassRateFloat))
	if index<=10:
		startThr = thrList[index]
	else:
		startThr = thrList[10]
	
	threshold = startThr
	result = reduceAndPredict(threshold)
	resultList.append(result)
	printResult(result)
	if optS:
		saveResult(optS,argResultFile,result)	
	
	if result[5] < minClassRate:
		headThr = threshold
		headResult = result
		bottomThr = threshold - maxOverallRRDelta
		if bottomThr <= 0:
			bottomThr = 0.01
		bottomResult = reduceAndPredict(bottomThr)
		resultList.append(bottomResult)
		printResult(bottomResult)
		if optS:
			saveResult(optS,argResultFile,bottomResult) 
		while (bottomResult[5] < minClassRate-0.1) and (decrementsStep < maxThresholdDecrementsStep):
			headThr = bottomThr
			headResult = bottomResult
			bottomThr = bottomThr - maxOverallRRDelta	
			if bottomThr <= 0:
				bottomThr = 0.01
			bottomResult = reduceAndPredict(bottomThr)
			resultList.append(bottomResult)
			printResult(bottomResult)
			if optS:
				saveResult(optS,argResultFile,bottomResult)
			decrementsStep += 1
			if bottomThr == headThr:
				decrementsStep = maxThresholdDecrementsStep
				threshold = (bottomThr + headThr)/2	
		result = bottomResult 
	else:
		bottomThr = threshold
		bottomResult = result
		headThr = threshold + maxOverallRRDelta
		headResult = reduceAndPredict(headThr)
		resultList.append(headResult)
		printResult(headResult)
		if optS:
			saveResult(optS,argResultFile,headResult)
		while (headResult[5] > minClassRate-0.1) and (decrementsStep < maxThresholdDecrementsStep):
			bottomThr = headThr
			bottomResult = headResult
			headThr = headThr + maxOverallRRDelta	
			headResult = reduceAndPredict(headThr)
			resultList.append(headResult)
			printResult(headResult)
			if optS:
				saveResult(optS,argResultFile,headResult)
			decrementsStep += 1 
		result = headResult
		
        while ( (result[5]>minClassRate+0.1) or (result[5]<minClassRate-0.1) ) and (headThr-bottomThr>minOverallRRDelta):
		threshold = (bottomThr + headThr)/2
		result = reduceAndPredict(threshold)
		resultList.append(result)
		printResult(result)
		if optS:
			saveResult(optS,argResultFile,result) 
		if result[5] < minClassRate-0.1:
			headThr = threshold	
		elif result[5] > minClassRate+0.1:
			bottomThr = threshold
			
        # End while
	
	#Find the best threshold guaranteeing the required classification rate
	print
	print 'Searching for maximal reduction guaranteeing the following class rate: %.3f%%' % (minClassRate) 
	print
	
	reduceRate=[]
	for line in resultList:
		if line[5]>=minClassRate:
			data=line[:]
			reduceRate.append(data[:])
	
	if len(reduceRate)>0:
        	print
        	print 'Results:'
        	max = getMaxPos(reduceRate, 1)
        	print 'Overall:   [ Red. rate: %.3f ] [ SVs: %d ]' % (max[1], max[2])
        	max = getMaxPos(reduceRate, 3)
        	print 'Summed up: [ Red. rate: %.3f ] [ SVs: %d ]' % (max[3], max[4])
        	print 'Params: [ Threshold: %s ]' % (str(max[0]))
        	print 'Class. rate: '+str(max[5])+'%%'
	
        print 'Finished!'

        totalTime = time.localtime(time.time()-startTime)
        print 'Total time: %d:%d:%d' % (totalTime[3]-1,totalTime[4],totalTime[5]) 
	
        print
        sys.exit()

main()


