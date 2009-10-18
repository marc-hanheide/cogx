#! /usr/bin/env python
# +---------------------+------------------------------+
# | SVReduction Test    | Author: Andrzej Pronobis     |
# | Version 0.1         | E-mail: pronobis@nada.kth.se |
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
# +----------------------------------------------------+

# ------------------------------------------------------
# Parameters
# ------------------------------------------------------

# Path to the directory containing the libSVM executables. 
# The path can be either absolute or relative to the 
# directory of the script.
paramLibSVMDir = '.' 

# If the following variable is set to false, the script
# will print the numbers only (without description) in
# the following order: 
# <threshold> <overall_red_rate> <summedup_red_rate> <class_rate>
#paramVerbose = True


minThreshold = 0.00000001
minThresholdStep = 0.00001
minOverallRRDelta = 0.5
maxOverallRRDelta = 2.0
thresholdStepMultiplier = 2.0

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
        print lines        
        exitWithError('Couldn\'t execute svm-reduce!')
    # Process results
    overallLine = findInList(lines, '* Overall')
    summedupLine = findInList(lines, '* Summed up')
    
    if len(summedupLine) == 0:
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
	exitWithHelp('Incorrect path to the training set.')
	
    if not os.path.isfile(argInputModel):
	exitWithHelp('Incorrect path to the input model.')

    if not os.path.isfile(argTestSet):
	exitWithHelp('Incorrect path to the test set.')


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
    
        # Find the starting threshold value
        print
        print 'Checking the initial classification rate and number of support vectors:'
        sys.stdout.flush()
        result = reduceAndPredict(1e-20)
        printResult(result)
        if optS:
            saveResult(optS, argResultFile, result)
            
        print
        print 'Searching for the initial threshold value: '
        sys.stdout.flush()

        threshold = minThreshold
        result = reduceAndPredict(threshold)
        startResult = result
        printResult(result)

        while result[3] == 0: # the summed up RR always changes first
            threshold *= 10.0
            startResult=result
            result = reduceAndPredict(threshold)
            printResult(result)
        
        print 'Found:'
        printResult(startResult)
        if optS:
            saveResult(optS, argResultFile, startResult)
        
        print
        print 'Finding the dependency between the reduction rate and the classification rate:'
        thresholdStep = max(minThresholdStep, startResult[0]) 
        print 'Initial step: %s' % str(thresholdStep)
        threshold = startResult[0]+thresholdStep
        oldThreshold = threshold
        result = startResult
        minClassRate = startResult[5]*(float(argMinClassRate)/100.0)
        
        multiplied = False
        waitForRR = -1
        thrList=[]
        while result[5] > minClassRate:
            # Get results for the threshold
            oldResult = result
            tmp = findThresholdInList(thrList, threshold)
            if tmp[0]==-1:
                result = reduceAndPredict(threshold)
                thrList.append(result)
            else:
                print 'Mem:',
                result = tmp

            printResult(result)
            
            # Decide what to do
            # The reduction rate delta is to small and it was not to big before
            # print str(result[1])+' '+str(oldResult[1])
            if (result[1]-oldResult[1] < minOverallRRDelta) and (result[1]>=waitForRR):
                thresholdStep *= thresholdStepMultiplier
                multiplied = True
                waitForRR = -1
                if optS:
                    saveResult(optS, argResultFile, result)
                print 'The step is to small. Increasing to %s.' % (str(thresholdStep))

            # The reduction rate delta is to big
            elif result[1]-oldResult[1] > maxOverallRRDelta:
                multiplied = False
                waitForRR = result[1]           # Do not increase the step until we reach the result[1] rr
                threshold = oldThreshold        # Bring the previous threshold back
                thresholdStep /= thresholdStepMultiplier # Decrease the step
                result=oldResult
                print 'The step is to big. Discarding the result and decreasing the step to %s.' % (str(thresholdStep))

            # Everything is fine, just save the results
            else:
                if optS:
                    saveResult(optS, argResultFile, result)

            
            oldThreshold = threshold
            threshold += thresholdStep

        # End while
        
        print 'Finished!'

        totalTime = time.localtime(time.time()-startTime)
        print 'Total time: %d:%d:%d' % (totalTime[3]-1,totalTime[4],totalTime[5]) 
        
        print
        sys.exit()



main()


