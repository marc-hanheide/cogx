#! /usr/bin/env python

version="1.0"

# -------------------------
# Imports
# -------------------------
import sys
import getopt
import pylib
import copy


# -------------------------
# Command line options
# -------------------------
class Options:
    optC=False
    optK=False
    optU=False
    optE=False
    optG=False
    optW=False
    optP=False
    optM=False
    optR=False
    optT=False
    optI=False
    optX=False
    confThreshold=0.5
    optA=False
    maxconfThreshold=1.0
    optS=False
    step=0.1
    optF=False
    matlabFileName=""
    matlabPrefix=""
    targetFileNames=[]
    dataFileNames=[]
    confidenceFile=False
    verbose=False


# -------------------------
# Prints command line help
# -------------------------
def printCommandLineHelp():
    o = Options()
    print "Analyse Classification Results Script "+version
    print "Usage: AnClassResults.py [-c] [-k] [-u] [-e] [-g] [-w] [-p] [-m] [-r]"
    print "[-i] [-A] [-v] [-t <conf_thresh>] [-a <max_conf_thresh>] [-s <step>]"
    print "[-f <filename>] [-n <name_prefix>]"
    print "(<target_file> <output_file>|<confidence_file>)* "
    print ""
    print "This script analyses the results produced by svm-predict. "
    print "The analysis is based on the output, confidence and target files. "
    print "The script allows to obtain: "
    print " -c : the standard classification rate for 1 best match"
    print "      (average over input files)"
    print " -k : the classification rates for K best matches"
    print "      (average over input files)"
    print " -u : number of samples for which the classifier was confident"
    print "      and unconfident (sum over all input files)"
    print " -e : classification rate for the best match excluding predictions"
    print "      that are not confident enough (average over all input files)"
    print " -g : classification rate for the best match and not confident "
    print "      prediction treated as correct (average over all input files)"
    print " -w : classification rate for matches within some confidence "
    print "      threshold (e.g. if the classifier sais its either class1 or "
    print "      class2 and one of them is correct it's counted as success) "
    print "      (average over all input files)"
    print " -p : number and percentage of samples for which the classifier "
    print "      was not confident enough that were in fact properly classified"
    print "      (sum over all input files)"
    print " -m : number and percenteage of samples for which the classifier "
    print "      was not confident enough that were in fact misclassified "
    print "      (sum over all input files)"
    print " -r : ratio between the number of samples for which classifier "
    print "      was not confident enough but were in fact classifier "
    print "      properly to those that were in fact misclassified (computed"
    print "      for sum over all input files)"
    print " -i : statistic showing which match is most often the proper"
    print "      one when the classifier is not confident (sum over all"
    print "      input files)"
    print " -x : confusion matrix"
    print " -A : all of the above"
    print ""
    print "The confidence threshold can be either fixed (-t <conf_thresh>, "
    print "default %g) or automatically modified within the range" % o.confThreshold
    print "[0;<max_conf_thresh>] (-a <max_conf_thresh>, default %g)" % o.maxconfThreshold
    print "with a step <step> (-s <step>, default %g) in order to" % o.step
    print "produce a trace."
    print ""
    print "The results are printed to the standard output and can be saved"
    print "in a Matlab format to a file (-f <filename>). The variables names"
    print "might be prefixed if -n <name_prefix> is used. If the -v (verbose)"
    print "option is used, additional information is printed."
    print ""
    print "The options must be followed by a list of pairs of filenames"
    print "of target and output/confidence files."


# -------------------------
# Exit with help and error
# -------------------------
def exitWithHelp(error):
    printCommandLineHelp()
    print "----------------------------------------------------------------"
    print
    print "Error: "+error
    print
    sys.exit()    


# -------------------------
# Exit with error
# -------------------------
def exitWithError(error):
    print
    print "----------------------------------------------------------------"
    print
    print "Error: "+error
    print
    sys.exit()    


# -------------------------
# Check and decode command line args
# -------------------------

def decodeCommandLineArgs(optList, args):
    # Check the number of arguments
    if len(args)<2:
        exitWithHelp("Missing command line arguments!")
    if len(args)%2!=0:
        exitWithHelp("Please provide both target and output/confidence file!")
    
    # Decode the options
    
    options = Options()
    for o in optList:
        if o[0]=='-c':
            options.optC=True
        elif o[0]=='-k':
            options.optK=True
        elif o[0]=='-u':
            options.optU=True
        elif o[0]=='-e':
            options.optE=True
        elif o[0]=='-g':
            options.optG=True
        elif o[0]=='-w':
            options.optW=True
        elif o[0]=='-p':
            options.optP=True
        elif o[0]=='-m':
            options.optM=True
        elif o[0]=='-r':
            options.optR=True
        elif o[0]=='-i':
            options.optI=True
        elif o[0]=='-x':
            options.optX=True
        elif o[0]=='-A':
            options.optC=True
            options.optK=True
            options.optU=True
            options.optE=True
            options.optG=True
            options.optW=True
            options.optP=True
            options.optM=True
            options.optR=True
            options.optI=True         
            options.optX=True         
        elif o[0]=='-v':
            options.verbose=True
        elif o[0]=='-t':
            try:
                options.optT=True
                options.confThreshold=float(o[1])
                if options.confThreshold<0:
                    raise Exception()
            except:
                exitWithHelp("Incorrect value of the -t option!")
        elif o[0]=='-a':
            try:
                options.optA=True
                options.maxconfThreshold=float(o[1])
                if options.maxconfThreshold<0:
                    raise Exception()
            except:
                exitWithHelp("Incorrect value of the -a option!")
        elif o[0]=='-s':
            try:
                options.optS=True
                options.step=float(o[1])
                if options.step<=0:
                    raise Exception()
            except:
                exitWithHelp("Incorrect value of the -s option!")
                
        elif o[0]=='-f':
            options.optF=True
            options.matlabFileName=o[1]
        elif o[0]=='-n':
            options.matlabPrefix=o[1]
        else:
            exitWithHelp("Incorrect command line option!")

    # Decode the arguments
    for i in range(0, len(args), 2):
        options.targetFileNames.append(args[i])
        options.dataFileNames.append(args[i+1])
        
    # Is there any task selected?
    if not (options.optC or options.optK or options.optU or options.optE or options.optG or options.optW or options.optP or options.optM or options.optI):
        exitWithHelp("No task was selected!")
    
    # Is both -t and -a present?
    if (options.optT and options.optA):
        exitWithHelp("Only one of the options -t and -a can be used at the same time!")

    return options


# -------------------------
# Checks whether the files can be opened and 
# checks if the data file is the confidence file
# -------------------------
def checkFiles(options):
    
    # Check if the files exist
    for i in options.targetFileNames:
        if not pylib.fileExists(i):
            exitWithHelp("The target file \'"+i+ "\' doesn't exist!")
    for i in options.dataFileNames:
        if not pylib.fileExists(i):
            exitWithHelp("The data file \'"+i+ "\' doesn't exist!")
    
    # Open the data files and see what kind of file are they
    options.confidenceFile=5 # Some value that is different from 1 and 0
    for i in options.dataFileNames:
        try:
            df=open(i, 'r')
            line = df.readline()
            tmp=pylib.findSubStr(line, ':')!=-1
            df.close()
            if options.confidenceFile==5:
                options.confidenceFile=tmp
            else:
                if options.confidenceFile!=tmp:
                    exitWithHelp("All data files must be of the same type!")
        except:
            exitWithHelp("Cannot read the data file \'"+i+ "\'!")
    
    # Is the confidence file when its required?
    if (not options.confidenceFile) and (options.optK or options.optU or options.optE or options.optW or options.optP or options.optM or options.optR or options.optI):
        exitWithHelp("Selected tasks require confidence file!")




# -------------------------
# Print options
# -------------------------
def printOptions(opt):
    print "Selected tasks:"
    if opt.optC:
        print " - (c) Obtain std. classification rate for the best match"
    if opt.optK:
        print " - (c) Obtain classification rates for K best matches"
    if opt.optU:
        print " - (u) Obtain number of confident and not confident matches"
    if opt.optE:
        print " - (e) Obtain class. rate excluding not confident matches"
    if opt.optG:
        print " - (g) Obtain class. rate for unconfident trated as correct"
    if opt.optW:
        print " - (w) Obtain class. rate for matches within conf. range"
    if opt.optP:
        print " - (p) Obtain perc. of not confident properly classified samples"
    if opt.optM:
        print " - (m) Obtain perc. of not confident misclassified samples"
    if opt.optR:
        print " - (r) Obtain ratio between not conf. prop. classified & misclassified"
    if opt.optI:
        print " - (i) Statistic showing which match is the right one for not confident"
    print ""
    print "Confidence threshold:",
    if opt.optT:
        print "fixed " + str(opt.confThreshold)
    elif opt.optA:
        print "variable [0," + str(opt.maxconfThreshold) +"] with a step " + str(opt.step)
    else:
        print "not used"
    print ""
    print "Input files:"
    for i in range(0,len(opt.targetFileNames)):
        print " - T:"+opt.targetFileNames[i]
        if opt.confidenceFile:
            print "   C:"+opt.dataFileNames[i]
        else:
            print "   O:"+opt.dataFileNames[i]
    if opt.optF:
        print 
        print "Matlab output file: "+opt.matlabFileName


# -------------------------
# Prints description of the results shown
# -------------------------
def printResultsDescr(options):
    i=1
    if options.optU:
        print " %d) Number of samples for which the classifier was confident" %(i)
        i+=1
        print " %d) Number of samples for which the classifier was not confident" %(i)
        i+=1
    if options.optE:
        print " %d) Classification rate excluding not confident predictions" % (i)
        i+=1
    if options.optG:
        print " %d) Classification rate for unconfident treated as correct" % (i)
        i+=1
    if options.optW:
        print " %d) Class rate for matches within confidence range" % (i)
        i+=1
    if options.optP:
        print " %d) Number of unconfident but properly classified samples" % (i)
        i+=1
        print " %d) Percentage of unconfident but properly classified samples" % (i)
        i+=1
    if options.optM:
        print " %d) Number of unconfident and missclassified samples" % (i)
        i+=1
        print " %d) Percentage of unconfident and missclassified samples" % (i)
        i+=1
    if options.optR:
        print " %d) Ratio between unconfident properly class. and missclassified" % (i)
        i+=1
    

# -------------------------
# Reads a target file
# -------------------------
def readTargetFile(fileName):
    lines=[]
    try:
        f=open(fileName, 'r')
        lines=f.readlines()
        f.close()
    except:
        exitWithError("Cannot open the target file \'" + fileName + "\'!")
    target=[]
    try:
        for l in lines:
            target.append(int(l))
    except:
        exitWithError("Error in the target file \'" + fileName + "\'!")
    return target


# -------------------------
# Reads a output file
# -------------------------
def readOutputFile(fileName):
    lines=[]
    try:
        f=open(fileName, 'r')
        lines=f.readlines()
        f.close()
    except:
        exitWithError("Cannot open the output file \'" + fileName + "\'!")
    output=[]
    try:
        for l in lines:
            output.append(int(l))
    except:
        exitWithError("Error in the output file \'" + fileName + "\'!")
    return output


# -------------------------
# Reads a confidence file
# -------------------------
def readConfidenceFile(fileName):
    lines=[]
    try:
        f=open(fileName, 'r')
        lines=f.readlines()
        f.close()
    except:
        exitWithError("Cannot open the confidence file \'" + fileName + "\'!")
    
    confidence=[]
    try:
        for l in lines:
            pieces=l.split(' ')
            c=[]
            for p in pieces:
                if len(p.strip())>0:
                    tmp=p.split(':')
                    c.append((int(tmp[0]),float(tmp[1])))
            confidence.append(c)
    except:
        exitWithError("Error in the confidence file \'" + fileName + "\'!")
    return confidence


# -------------------------
# Converts a confidence info to output info
# -------------------------
def confidenceToOutput(confidence):
    output=[]
    for c in confidence:
        output.append(c[0][0])
    return output



# -------------------------
# Reads all data from the files
# -------------------------
def readAllData(options, file_nr):
    # Read target file
    target=readTargetFile(options.targetFileNames[file_nr])
    # Read data file
    output=[]
    confidence=[]
    if options.confidenceFile:
        confidence=readConfidenceFile(options.dataFileNames[file_nr])
        output=confidenceToOutput(confidence)
    else:
        output=readOutputFile(options.dataFileNames[file_nr])
    # Check if the length is good
    if len(output)!=len(target):
        exitWithError("The lengths of target and data file don't match!")
    # Return
    return target, output, confidence


# -------------------------
# Finds the classification rate for K best matches
# -------------------------
def getClassRate(target, output, confidence, K):
    classRate=0
    # K=1
    if K==1:
        for i in range(0, len(target)):
            if target[i]==output[i]:
                classRate=classRate+1
        classRate=float(classRate)/float(len(target))*100.0
    # K!=1
    else:
        for i in range(0, len(target)):
            for j in range(0, K):
                if target[i]==confidence[i][j][0]:
                    classRate=classRate+1
                    break
        classRate=(float(classRate)/float(len(target)))*100
    # Return
    return classRate;


# -------------------------
# Number of samples for which the classifier was confident
# and unconfident
# -------------------------
def getConfidentCount(confidence, confThreshold):
    unconfidentCount=0
    confidentCount=0
    
    for i in range(0, len(confidence)):
        if confidence[i][0][1]>=confThreshold:
            confidentCount+=1
        else:
            unconfidentCount+=1

    return confidentCount, unconfidentCount


# -------------------------
# Classification rate for the best match excluding
# predictions that are not confident enough
# -------------------------
def getClassRateConfident(target, confidence, confThreshold):
    classRateConfident=0
    confidentCount=0
    
    for i in range(0, len(target)):
        if confidence[i][0][1]>=confThreshold:
            if target[i]==confidence[i][0][0]:
                classRateConfident+=1
            confidentCount+=1
    if confidentCount>0:
        classRateConfident=(float(classRateConfident)/float(confidentCount))*100.0
    else: 
        classRateConfident=100.0

    return classRateConfident


# -------------------------
# Classification rate for the best match and
# unconfident prediction treated as correct
# -------------------------
def getClassRateUnconfidentCorrect(target, confidence, confThreshold):
    classRateUnconfidentCorrect=0
    
    for i in range(0, len(target)):
        if not ((confidence[i][0][1]>=confThreshold) and (target[i]!=confidence[i][0][0])):
            classRateUnconfidentCorrect+=1
    
    return (float(classRateUnconfidentCorrect)/float(len(target)))*100.0



# -------------------------
# Classification rate for matches within some confidence 
# threshold (e.g. if the classifier sais its either class1 or 
# class2 and one of them is correct it's counted as success) 
# -------------------------
def getFuzzyClassRate(target, confidence, confThreshold):
    fuzzyClassRate=0
    for i in range(0, len(target)):
        if target[i]==confidence[i][0][0]: # The first answer was good
            fuzzyClassRate+=1
            continue
        for j in range(1, len(confidence[0])):
            if confidence[i][j][1]<confThreshold:
                if target[i]==confidence[i][j][0]:
                    fuzzyClassRate+=1
                    break
    return (float(fuzzyClassRate)/float(len(target)))*100.0


# -------------------------
# Returns the number of samples for which the classifier
# was not confident enought but were properly classified
# -------------------------
def getUnconfidentProperlyClassified(target, confidence, confThreshold):
    unconfidentCount=0;
    unconfPropClassCount=0;
    
    for i in range(0, len(target)):
        if confidence[i][0][1]<confThreshold:
            if target[i]==confidence[i][0][0]:
                unconfPropClassCount+=1
            unconfidentCount+=1

    return unconfPropClassCount, unconfidentCount
    

# -------------------------
# Returns the number of samples for which the classifier
# was not confident enought and were missclassified
# -------------------------
def getUnconfidentMissClassified(target, confidence, confThreshold):
    unconfidentCount=0;
    unconfMissClassCount=0;

    for i in range(0, len(target)):
        if confidence[i][0][1]<confThreshold:
            if target[i]!=confidence[i][0][0]:
                unconfMissClassCount+=1
            unconfidentCount+=1

    return unconfMissClassCount, unconfidentCount


# -------------------------
# Statistic showing which match is most often the proper
# one when the classifier is not confident
# -------------------------
def getUnconfStatistics(target, confidence, confThreshold):
    # Initialize
    unconfidentCount=0
    unconfStatistics=[]
    for i in range(0, len(confidence[0])):
        unconfStatistics.append(0)
    # Find the statistics
    for i in range(0, len(target)):
        if confidence[i][0][1]<confThreshold:
            unconfidentCount+=1
            for j in range(0, len(confidence[0])):
                if target[i]==confidence[i][j][0]:
                    unconfStatistics[j]+=1
                    break
    return unconfStatistics, unconfidentCount
    
    
# -------------------------
# Returns confusion matrix organized as follows:
# Rows: target, Columns: output
# -------------------------
def getConfusionMatrix(target, output):
    nrClass=max(max(target), max(output))
    confMatrix=[]
    # Initialize the matrix
    for i in range(nrClass):
        row=[]
        for j in range(nrClass):
            row.append(0)
        confMatrix.append(row)
    # Get the matrix
    for i in range(len(target)):
        confMatrix[target[i]-1][output[i]-1]+=1
    # Return the matrix
    return confMatrix

# -------------------------
# Prints horizontal line of the results table
# -------------------------
def printResultsHorizLine(options):
    pylib.printNoNewLine("+---------+")
    if options.optU:
        pylib.printNoNewLine("------+------+")
    if options.optE:
        pylib.printNoNewLine("---------+")
    if options.optG:
        pylib.printNoNewLine("---------+")
    if options.optW:
        pylib.printNoNewLine("---------+")
    if options.optP:
        pylib.printNoNewLine("------+---------+")
    if options.optM:
        pylib.printNoNewLine("------+---------+")
    if options.optR:
        pylib.printNoNewLine("--------+")
    print


# -------------------------
# Prints header of the results table
# -------------------------
def printResultsHeader(options):
    i=1
    print "| Conf.T. |" ,
    if options.optU:
        print " %2d  |  %2d  |"% (i,i+1),
        i+=2
    if options.optE:
        print "  %2d    |" %(i),
        i+=1
    if options.optG:
        print "  %2d    |" %(i),
        i+=1
    if options.optW:
        print "  %2d    |" %(i),
        i+=1
    if options.optP:
        print " %2d  |   %2d    |" %(i,i+1),
        i+=2
    if options.optM:
        print " %2d  |   %2d    |" %(i,i+1),
        i+=2
    if options.optR:
        print "  %2d   |" %(i),
        i+=1
    print



# -------------------------
# Main
# -------------------------
def main():

    # Decode command line arguments
    try:
        optList, args = getopt.getopt(sys.argv[1:], 'cxkegwpuvmiArt:a:s:f:n:')
    except:
        exitWithHelp("Incorrect command line options!")
    options=decodeCommandLineArgs(optList, args)
    checkFiles(options)
    
    # Print options
    if options.verbose:
        print "----------------------------------------------------------------"
        print " Analyse Classification Results Script "+version
        print "----------------------------------------------------------------"
        print
        print "----------------------------------------------------------------"
        print " Selected options"
        print "----------------------------------------------------------------"
        printOptions(options)
        print


    # Read input files
    targets=[]
    outputs=[]
    confidences=[]
    for fi in range(0, len(options.targetFileNames)):
        t, o, c = readAllData(options, fi)
        targets.append(t)
        outputs.append(o)
        confidences.append(c)


    # ---------------------------------------------------------
    # Get values independent from the confidence threshold
    # ---------------------------------------------------------
    # Init vars holding values for all files
    ALLclassRate=0
    ALLclassRatesK=[]
    ALLconfusionMatrix=[]
    ALLconfusionMatrixPerc=[]
    # Process input files
    for fi in range(0, len(options.targetFileNames)):
        # Get data for current file
        target = targets[fi]
        output = outputs[fi]
        confidence = confidences[fi]
        length=len(target)

        # Init vars
        classRate=-1
        classRatesK=[]
        confusionMatrix=[]

        # The classification rate for the best match
        if options.optC:
            classRate=getClassRate(target, output, confidence, 1)

        # The classification rate for K best matches
        if options.optK:
            for i in range(0, len(confidence[0])):
                classRatesK.append(getClassRate(target, output, confidence, i+1))

        # The confusion matrix
        if options.optX:
            confusionMatrix=getConfusionMatrix(target, output)

        # Add to the overall results
        ALLclassRate+=classRate
        if ALLclassRatesK==[]:
            ALLclassRatesK=copy.deepcopy(classRatesK)
        else:
            for i in range(0, len(ALLclassRatesK)):
                ALLclassRatesK[i]+=classRatesK[i]
        if options.optX:
            # Confusion matrix
            if (fi==0):
                ALLconfusionMatrix=copy.deepcopy(confusionMatrix)
            else:
                # Check if the size must be changed
                sizediff=max(len(confusionMatrix)-len(ALLconfusionMatrix),0)
                for i in range(len(ALLconfusionMatrix)):
                    for s in range(sizediff):
                        ALLconfusionMatrix[i].append(0)
                for s in range(sizediff):
                    row=[]
                    for i in range(len(confusionMatrix)):
                        row.append(0)
                    ALLconfusionMatrix.append(row)
                # Sum up the matrices
                for i in range(len(confusionMatrix)):
                    for j in range(len(confusionMatrix)):
                        ALLconfusionMatrix[i][j]+=confusionMatrix[i][j]
                    

    # Perform final computations
    ALLclassRate/=len(options.targetFileNames)
    for i in range(0, len(ALLclassRatesK)):
        ALLclassRatesK[i]/=len(options.targetFileNames)
    if options.optX:
        ALLconfusionMatrixPerc=copy.deepcopy(ALLconfusionMatrix)
        for i in range(0, len(ALLconfusionMatrix)):
            if sum(ALLconfusionMatrix[i])>0:
                for j in range(0, len(ALLconfusionMatrix[0])):
                    ALLconfusionMatrixPerc[i][j]= \
                    float(ALLconfusionMatrix[i][j])*100.0/float(sum(ALLconfusionMatrix[i]))


    # ----------------------------------------------------------------
    # Get values dependent on the confidence threshold
    # ----------------------------------------------------------------
    # Init result tables for all confidence thresholds
    TABconfidentCount=[]
    TABunconfidentCount=[]
    TABclassRateConfident=[]
    TABclassRateUnconfidentCorrect=[]
    TABfuzzyClassRate=[]
    TABunconfPropClassCount=[]
    TABunconfMissClassCount=[]
    TABunconfPropClassPerc=[]
    TABunconfMissClassPerc=[]
    TABunconfRatio=[]
    TABunconfStatistics=[]
    TABconfThresholds=[]
    
    # Init table with confidence thresholds
    if (options.optT):
        TABconfThresholds.append(options.confThreshold)
    elif (options.optA):
        cl=0.0
        while cl<options.maxconfThreshold+options.step:
            TABconfThresholds.append(cl)
            cl+=options.step
        
    # Iterate through the confidence thresholds
    for confThreshold in TABconfThresholds:

        # Init variables holding final values for all files
        ALLconfidentCount=0
        ALLunconfidentCount=0
        ALLclassRateConfident=0
        ALLclassRateUnconfidentCorrect=0
        ALLfuzzyClassRate=0
        ALLunconfPropClassCount=0
        ALLunconfMissClassCount=0
        ALLunconfStatistics=[]
    
        # Process input files
        for fi in range(0, len(options.targetFileNames)):
            # Get data for current file
            target = targets[fi]
            output = outputs[fi]
            confidence = confidences[fi]
            length=len(target)
    
            # Init all vars
            confidentCount=-1
            unconfidentCount=-1
            classRateConfident=-1
            classRateUnconfidentCorrect=-1
            fuzzyClassRate=-1
            unconfPropClassCount=-1
            unconfMissClassCount=-1
            unconfStatistics=[]
                
            # Number of samples for which the classifier was confident
            # and unconfident
            if options.optU:
                confidentCount, unconfidentCount = \
                getConfidentCount(confidence, confThreshold)
    
            # Classification rate for the best match excluding
            # predictions that are not confident enough
            if options.optE:
                classRateConfident = getClassRateConfident(target, confidence, confThreshold)

            # Classificaiton rate for unconfident treated as correct
            if options.optG:
                classRateUnconfidentCorrect = \
                  getClassRateUnconfidentCorrect(target, confidence, confThreshold)


            # Classification rate for matches within some confidence 
            # threshold (e.g. if the classifier sais its either class1 or 
            # class2 and one of them is correct it's counted as success) 
            if options.optW:
                fuzzyClassRate=getFuzzyClassRate(target, confidence, confThreshold)
    
            # Number and percenteage of samples for which the classifier was not 
            # confident enough that were in fact properly classified (-p)
            # Number and percenteage of samples for which the classifier was not 
            # confident enough that were in fact misclassified (-m)
            # Ratio between the number of samples for which classifier "
            # was not confident enough but were in fact classifier "
            # properly to those that were in fact misclassified (-r"
    
            if options.optP or options.optR:
                # Get the number of unconfident & properly classified
                unconfPropClassCount, unconfidentCount= \
                getUnconfidentProperlyClassified(target, confidence, confThreshold)
    
            if options.optM or options.optR:
                # Get the number of unconfident & missclassified
                unconfMissClassCount, unconfidentCount= \
                getUnconfidentMissClassified(target, confidence, confThreshold)
    
    
            # Statistic showing which match is most often the proper
            # one when the classifier is not confident
            if options.optI:
                unconfStatistics, unconfidentCount = \
                getUnconfStatistics(target, confidence, confThreshold) 
    
            # Add to the overall results
            ALLconfidentCount+=confidentCount
            ALLunconfidentCount+=unconfidentCount
            ALLclassRateConfident+=classRateConfident
            ALLclassRateUnconfidentCorrect+=classRateUnconfidentCorrect
            ALLfuzzyClassRate+=fuzzyClassRate
            ALLunconfPropClassCount+=unconfPropClassCount
            ALLunconfMissClassCount+=unconfMissClassCount
            if ALLunconfStatistics==[]:
                ALLunconfStatistics=copy.deepcopy(unconfStatistics)
            else:
                for i in range(0, len(ALLunconfStatistics)):
                    ALLunconfStatistics[i]+=unconfStatistics[i]
    
            # END FOR over files
    
    
        # Perform final computations
        ALLclassRateConfident/=len(options.targetFileNames)
        ALLclassRateUnconfidentCorrect/=len(options.targetFileNames)
        ALLfuzzyClassRate/=len(options.targetFileNames)
    
        if options.optP:
            # Get the percentage
            if ALLunconfidentCount>0:
                ALLunconfPropClassPerc= \
                (float(ALLunconfPropClassCount)/float(ALLunconfidentCount))*100.0
            else:
                ALLunconfPropClassPerc=0
    
        if options.optM:
            # Get the percentage
            if ALLunconfidentCount>0:
                ALLunconfMissClassPerc= \
                (float(ALLunconfMissClassCount)/float(ALLunconfidentCount))*100.0
            else:
                ALLunconfMissClassPerc=0
    
        if options.optR:
            # Get the ratio
            if ALLunconfidentCount>0:
                if ALLunconfMissClassCount>0:
                    ALLunconfRatio=\
                    (float(ALLunconfPropClassCount)/float(ALLunconfMissClassCount))
                else:
                    ALLunconfRatio=0
            else:
                ALLunconfRatio=0
    
        # Save the results to tables
        TABconfidentCount.append(ALLconfidentCount)
        TABunconfidentCount.append(ALLunconfidentCount)
        TABclassRateConfident.append(ALLclassRateConfident)
        TABclassRateUnconfidentCorrect.append(ALLclassRateUnconfidentCorrect)
        TABfuzzyClassRate.append(ALLfuzzyClassRate)
        TABunconfPropClassCount.append(ALLunconfPropClassCount)
        TABunconfMissClassCount.append(ALLunconfMissClassCount)
        TABunconfPropClassPerc.append(ALLunconfPropClassPerc)
        TABunconfMissClassPerc.append(ALLunconfMissClassPerc)
        TABunconfRatio.append(ALLunconfRatio)
        TABunconfStatistics.append(ALLunconfStatistics)
        
        # END FOR iterating over confidence thresholds




    # ----------------------------------------------------------------
    # Print the final results
    # ----------------------------------------------------------------
    if options.verbose:
        print "----------------------------------------------------------------"
        print " Results"
        print "----------------------------------------------------------------"

    # Print the classification rates
    if options.optC:
        print "Classification rate: %7.5f" % (ALLclassRate)
        print
        
    if options.optK:
        print "Classification rates for K best matches:" 
        pylib.printNoNewLine('+')
        for i in range(0, len(ALLclassRatesK)):
            pylib.printNoNewLine('---------+')
        print
        print "|" ,
        for i in range(0, len(ALLclassRatesK)):
            print '  K%02d   |'%(i+1), 
        print
        pylib.printNoNewLine('+')
        for i in range(0, len(ALLclassRatesK)):
            pylib.printNoNewLine('---------+')
        print
        #for i in range(0,len(TABclassRatesK)):
        print "|",
        for j in range(0, len(ALLclassRatesK)):
            print '%7.3f |' %(ALLclassRatesK[j]),
        print
        pylib.printNoNewLine('+')
        for i in range(0, len(ALLclassRatesK)):
            pylib.printNoNewLine('---------+')
        print
        print


    # Print the main results
    if options.optU or options.optE or options.optW or options.optP or options.optM or \
       options.optR:
        print 'Results for various confidence thresholds:'
        printResultsDescr(options)
        print
        printResultsHorizLine(options)
        printResultsHeader(options)
        printResultsHorizLine(options)
        for i in range(0, len(TABconfThresholds)):
            print "| %7.4f |" % (TABconfThresholds[i]),
            if options.optU:
                print "%4d | %4d |" % (TABconfidentCount[i], TABunconfidentCount[i]),
            if options.optE:
                print "%7.3f |" % (TABclassRateConfident[i]),
            if options.optG:
                print "%7.3f |" % (TABclassRateUnconfidentCorrect[i]),
            if options.optW:
                print "%7.3f |" % (TABfuzzyClassRate[i]),
            if options.optP:
                print "%4d | %7.3f |" % (TABunconfPropClassCount[i], TABunconfPropClassPerc[i]),
            if options.optM:
                print "%4d | %7.3f |" % (TABunconfMissClassCount[i], TABunconfMissClassPerc[i]),
            if options.optR:
                print "%6.3f |" % (TABunconfRatio[i]),
            print
        printResultsHorizLine(options)
        print

    # Print the statistics
    if options.optI:
        print "Statistics showing which match was correct for unconfident samples:" 
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(TABunconfStatistics[0])):
            pylib.printNoNewLine('------+')
        print
        print "| Conf.T. |" ,
        for i in range(0, len(TABunconfStatistics[0])):
            print ' M%02d |'%(i+1), 
        print
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(TABunconfStatistics[0])):
            pylib.printNoNewLine('------+')
        print
        for i in range(0,len(TABunconfStatistics)):
            print "| %7.4f |" % (TABconfThresholds[i]),
            for j in range(0, len(TABunconfStatistics[0])):
                print '%4d |' %(TABunconfStatistics[i][j]),
            print
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(TABunconfStatistics[0])):
            pylib.printNoNewLine('------+')
        print
        print

    # Print the confusion matrix
    if options.optX:
        print "Confusion matrix:" 
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(ALLconfusionMatrix[0])):
            pylib.printNoNewLine('------+')
        print
        print "| Tgt/Out |" ,
        for i in range(0, len(ALLconfusionMatrix[0])):
            print ' C%02d |'%(i+1), 
        print
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(ALLconfusionMatrix[0])):
            pylib.printNoNewLine('------+')
        print
        for i in range(0,len(ALLconfusionMatrix)):
            print "|   C%02d   |" % (i+1),
            for j in range(0, len(ALLconfusionMatrix[0])):
                print '%4d |' %(ALLconfusionMatrix[i][j]),
            print
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(ALLconfusionMatrix[0])):
            pylib.printNoNewLine('------+')
        print
        print

        print "Confusion matrix in %:" 
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(ALLconfusionMatrix[0])):
            pylib.printNoNewLine('---------+')
        print
        print "| Tgt/Out |" ,
        for i in range(0, len(ALLconfusionMatrix[0])):
            print '  C%02d   |'%(i+1), 
        print
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(ALLconfusionMatrix[0])):
            pylib.printNoNewLine('---------+')
        print
        for i in range(0,len(ALLconfusionMatrix)):
            print "|   C%02d   |" % (i+1),
            for j in range(0, len(ALLconfusionMatrix[0])):
                print '%7.3f |' % (ALLconfusionMatrixPerc[i][j]),
            print
        pylib.printNoNewLine('+---------+')
        for i in range(0, len(ALLconfusionMatrix[0])):
            pylib.printNoNewLine('---------+')
        print
        print



    # ---------------------------------
    # Create the matlab file
    # ---------------------------------
    if options.optF:
        fm=open(options.matlabFileName, 'w')
        pylib.writeMatlabVector(fm, options.matlabPrefix+'ConfThreshold', \
                                TABconfThresholds)
        if options.optC:
            pylib.writeMatlabValue(fm, options.matlabPrefix+'ClassRate', ALLclassRate )
        if options.optK:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'ClassRatesK', ALLclassRatesK)
        if options.optX:
            pylib.writeMatlabMatrix(fm, options.matlabPrefix+'ConfusionMatrix', \
                                    ALLconfusionMatrix)
            pylib.writeMatlabMatrix(fm, options.matlabPrefix+'ConfusionMatrixPerc', \
                                    ALLconfusionMatrixPerc)
        if options.optU:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'ConfCount', TABconfidentCount)
            pylib.writeMatlabVector(fm, options.matlabPrefix+'UnconfCount', TABunconfidentCount)
        if options.optE:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'ClassRateConfident', \
                                    TABclassRateConfident)
        if options.optG:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'ClassRateUnconfidentCorrect', \
                                    TABclassRateUnconfidentCorrect)
        if options.optW:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'FuzzyClassRate', \
                                    TABfuzzyClassRate)
        if options.optP:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'UnconfPropClassCount', \
                                    TABunconfPropClassCount)
            pylib.writeMatlabVector(fm, options.matlabPrefix+'UnconfPropClassPerc', \
                                    TABunconfPropClassPerc)
        if options.optM:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'UnconfMissClassCount', \
                                    TABunconfMissClassCount)
            pylib.writeMatlabVector(fm, options.matlabPrefix+'UnconfMissClassPerc', \
                                    TABunconfMissClassPerc)
        if options.optR:
            pylib.writeMatlabVector(fm, options.matlabPrefix+'UnconfRatio', \
                                    TABunconfRatio)
        if options.optI:
            pylib.writeMatlabMatrix(fm, options.matlabPrefix+'UnconfStatistics', \
                                    TABunconfStatistics)
        fm.close()
  
    
main()
