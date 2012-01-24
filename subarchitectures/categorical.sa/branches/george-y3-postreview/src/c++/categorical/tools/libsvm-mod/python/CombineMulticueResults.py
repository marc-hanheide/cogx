#! /usr/bin/env python

version="0.9"

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
    confThreshold=0.5
    optA=False
    optO=False
    optT=False
    maxconfThreshold=1.0
    step=0.1
    optF=False
    matlabFileName=""
    targetFileNames=[]
    primaryDataFileNames=[]
    secondaryDataFileNames=[]
    outputFileNames=[]
    verbose=False


# -------------------------
# Prints command line help
# -------------------------
def printCommandLineHelp():
    o = Options()
    print "Combine Multi-cue Classification Results Script "+version
    print "Usage: CombineMulticueResults.py [-o] [-v] [-t <conf_thresh>]"
    print "[-a <max_conf_thresh>] [-s <step>] [-f <filename>]"
    print "(<target_file> <primary_confidence_file> <secondary_confidence_file>"
    print " <output_confidence_file>)* "
    print ""
    print "This script analyses the results produced by svm-predict for"
    print "multiple cues. The analysis is based on the confidence and target"
    print "files. The script uses the primary cue if it is confident enough and"
    print "the secondary cue (or cues if the second file is a result of DAS)"
    print "otherwise"
    print ""
    print "The confidence threshold can be either fixed (-t <conf_thresh>, "
    print "default %g) or automatically modified within the range" % o.confThreshold
    print "[0;<max_conf_thresh>] (-a <max_conf_thresh>, default %g)" % o.maxconfThreshold
    print "with a step <step> (-s <step>, default %g) in order to" % o.step
    print "produce a trace."
    print ""
    print "The results are printed to the standard output and can be saved"
    print "in a Matlab format to a file (-f <filename>). If the -v (verbose)"
    print "option is used, additional information is printed."
    print ""
    print "The options must be followed by a list of quadruples of filenames"
    print "of target, primary, secondary and output confidence files."
    print "The output files are used only if the -o switch is enabled."
    print "Output confidence file can be used only with -t option."


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
    if len(args)<4:
        exitWithHelp("Missing command line arguments!")
    if len(args)%4!=0:
        exitWithHelp("Please provide all target, primary, secondary and output confidence file!")
    
    # Decode the options
    options = Options()
    for o in optList:
        if o[0]=='-v':
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
                options.step=float(o[1])
                if options.step<=0:
                    raise Exception()
            except:
                exitWithHelp("Incorrect value of the -s option!")
                
        elif o[0]=='-f':
            options.optF=True
            options.matlabFileName=o[1]
        else:
            exitWithHelp("Incorrect command line option!")

    # Decode the arguments
    for i in range(0, len(args), 4):
        options.targetFileNames.append(args[i])
        options.primaryDataFileNames.append(args[i+1])
        options.secondaryDataFileNames.append(args[i+2])
        options.outputFileNames.append(args[i+3])
    
    # Is both -t and -a present or none of them?
    if (not (options.optT or options.optA)) or (options.optT and options.optA):
        exitWithHelp("One of the options -t or -a must be used!")

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
    for i in options.primaryDataFileNames:
        if not pylib.fileExists(i):
            exitWithHelp("The primary data file \'"+i+ "\' doesn't exist!")
    for i in options.secondaryDataFileNames:
        if not pylib.fileExists(i):
            exitWithHelp("The secondary data file \'"+i+ "\' doesn't exist!")
    
    # Open the data files and see what kind of file are they
    for i in options.primaryDataFileNames:
        try:
            df=open(i, 'r')
            line = df.readline()
            tmp=pylib.findSubStr(line, ':')!=-1
            df.close()
            if (not tmp):
                exitWithHelp("All files must be confidence files!")
        except:
            exitWithHelp("Cannot read the primary data file \'"+i+ "\'!")

    for i in options.secondaryDataFileNames:
        try:
            df=open(i, 'r')
            line = df.readline()
            tmp=pylib.findSubStr(line, ':')!=-1
            df.close()
            if (not tmp):
                exitWithHelp("All files must be confidence files!")
        except:
            exitWithHelp("Cannot read the secondary data file \'"+i+ "\'!")
    


# -------------------------
# Print options
# -------------------------
def printOptions(opt):
    print "Confidence threshold:",
    if opt.optT:
        print "fixed " + str(opt.confThreshold)
    elif opt.optA:
        print "variable [0," + str(opt.maxconfThreshold) +"] with a step " + str(opt.step)
    else:
        print "ERROR"
    print ""
    print "Input files:"
    for i in range(0,len(opt.targetFileNames)):
        print " - T:"+opt.targetFileNames[i]
        print "   Cp:"+opt.primaryDataFileNames[i]
        print "   Cs:"+opt.secondaryDataFileNames[i]
        print "   O:"+opt.outputFileNames[i]
    if opt.optF:
        print 
        print "Matlab output file: "+opt.matlabFileName

    

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
# Reads all data from the files
# -------------------------
def readAllData(options, file_nr):
    # Read target file
    target=readTargetFile(options.targetFileNames[file_nr])
    # Read data files
    primaryConfidence=readConfidenceFile(options.primaryDataFileNames[file_nr])
    secondaryConfidence=readConfidenceFile(options.secondaryDataFileNames[file_nr])
    # Check if the length is good
    if (len(primaryConfidence)!=len(target)) or (len(secondaryConfidence)!=len(target)):
        exitWithError("The lengths of target and data files don't match!")
    # Return
    return target, primaryConfidence, secondaryConfidence


# -------------------------
# Finds the classification rate for K best matches
# -------------------------
def getClassRate(target, confidence, K):
    classRate=0
    for i in range(0, len(target)):
        for j in range(0, K):
            if target[i]==confidence[i][j][0]:
                classRate=classRate+1
                break
    classRate=(float(classRate)/float(len(target)))*100.0
    # Return
    return classRate;


# -------------------------
# Combines two confidences: primary and secondary
# -------------------------
def combineConfidence(primaryConfidence, secondaryConfidence, confThreshold):
    outputConfidence=[]
    for i in range(0, len(primaryConfidence)):
        if primaryConfidence[i][0][1]>=confThreshold:
            outputConfidence.append(primaryConfidence[i])
        else:
            outputConfidence.append(secondaryConfidence[i])

    # Return
    return outputConfidence;


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
# Main
# -------------------------
def main():

    # Decode command line arguments
    try:
        optList, args = getopt.getopt(sys.argv[1:], 'vt:a:s:f:')
    except:
        exitWithHelp("Incorrect command line options!")
    options=decodeCommandLineArgs(optList, args)
    checkFiles(options)
    
    # Print options
    if options.verbose:
        print "----------------------------------------------------------------"
        print " Combine Multi-cue Classification Results Script "+version
        print "----------------------------------------------------------------"
        print
        print "----------------------------------------------------------------"
        print " Selected options"
        print "----------------------------------------------------------------"
        printOptions(options)
        print


    # Read input files
    targets=[]
    primaryConfidences=[]
    secondaryConfidences=[]
    for fi in range(0, len(options.targetFileNames)):
        t, pc, sc = readAllData(options, fi)
        targets.append(t)
        primaryConfidences.append(pc)
        secondaryConfidences.append(sc)

    # ---------------------------------------------------------
    # Get values independent from the confidence threshold
    # ---------------------------------------------------------
    # Init vars holding values for all files
    ALLclassRatePrimary=0
    ALLclassRateSecondary=0
    # Process input files
    for fi in range(0, len(options.targetFileNames)):
        # Get data for current file
        target = targets[fi]
        primaryConfidence = primaryConfidences[fi]
        secondaryConfidence = secondaryConfidences[fi]
        length=len(target)

        # Get primary and secondary classrates
        classRatePrimary=getClassRate(target, primaryConfidence, 1)
        classRateSecondary=getClassRate(target, secondaryConfidence, 1)

        # Add to the overall results
        ALLclassRatePrimary+=classRatePrimary
        ALLclassRateSecondary+=classRateSecondary

    # Perform final computations
    ALLclassRatePrimary/=len(options.targetFileNames)
    ALLclassRateSecondary/=len(options.targetFileNames)


    # ----------------------------------------------------------------
    # Get values dependent on the confidence threshold
    # ----------------------------------------------------------------
    # Init result tables for all confidence thresholds
    TABconfidentCountPrimary=[]
    TABunconfidentCountPrimary=[]
    TABclassRateCombined=[]
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
        ALLconfidentCountPrimary=0
        ALLunconfidentCountPrimary=0
        ALLclassRateCombined=0
    
        # Process input files
        for fi in range(0, len(options.targetFileNames)):
            # Get data for current file
            target = targets[fi]
            primaryConfidence = primaryConfidences[fi]
            secondaryConfidence = secondaryConfidences[fi]
            length=len(target)
    
            # Init all vars
            confidentCountPrimary=-1
            unconfidentCountPrimary=-1
            classRateCombined=-1
                
            # Number of samples for which the classifier was confident
            # and unconfident
            confidentCountPrimary, unconfidentCountPrimary = \
              getConfidentCount(primaryConfidence, confThreshold)

            # Classification rate for combined primary and secondary data
            combinedConfidence = \
              combineConfidence(primaryConfidence, secondaryConfidence, confThreshold)
            classRateCombined = getClassRate(target, combinedConfidence, 1)
    
            # Add to the overall results
            ALLconfidentCountPrimary+=confidentCountPrimary
            ALLunconfidentCountPrimary+=unconfidentCountPrimary
            ALLclassRateCombined+=classRateCombined
    
            # END FOR over files
    
    
        # Perform final computations
        ALLclassRateCombined/=len(options.targetFileNames)
  
        # Save the results to tables
        TABconfidentCountPrimary.append(ALLconfidentCountPrimary)
        TABunconfidentCountPrimary.append(ALLunconfidentCountPrimary)
        TABclassRateCombined.append(ALLclassRateCombined)
        
        # END FOR iterating over confidence thresholds



    # ----------------------------------------------------------------
    # Print the final results
    # ----------------------------------------------------------------
    if options.verbose:
        print "----------------------------------------------------------------"
        print " Results"
        print "----------------------------------------------------------------"

    # Print the classification rates
    print "Primary classification rate: %7.5f" % (ALLclassRatePrimary)
    print "Secondary classification rate: %7.5f" % (ALLclassRateSecondary)
    allSamplesCount=(TABconfidentCountPrimary[0]+TABunconfidentCountPrimary[0])
    print "Number of all samples: %d" % (allSamplesCount)
    print

    # Print confidence dependent results
    print "+--------------+--------------+----------------+------------+"
    print "| Conf Thrshld | Unconf Smpls | Unconf Smpls % | Class Rate |"
    print "+--------------+--------------+----------------+------------+"
    for i in range(0, len(TABconfThresholds)):
        print "|    %7.4f   |" % (TABconfThresholds[i]),
        print "    %4d     |" % (TABunconfidentCountPrimary[i]),
        print "    %7.3f    |" % \
          (float(TABunconfidentCountPrimary[i])*100.0/float(allSamplesCount)),
        print "  %7.3f  |" % (TABclassRateCombined[i]),
        print
    print "+--------------+--------------+----------------+------------+"


    # ---------------------------------
    # Create the matlab file
    # ---------------------------------
    if options.optF:
        fm=open(options.matlabFileName, 'w')
        pylib.writeMatlabVector(fm, 'ConfThreshold', TABconfThresholds)
        pylib.writeMatlabValue(fm, 'ClassRatePrimary', ALLclassRatePrimary )
        pylib.writeMatlabValue(fm, 'ClassRateSecondary', ALLclassRateSecondary )
        pylib.writeMatlabValue(fm, 'AllSamplesCount', allSamplesCount )
        pylib.writeMatlabVector(fm, 'UnconfCountPrimary', TABunconfidentCountPrimary)
        pylib.writeMatlabVector(fm, 'ClassRateCombined', TABclassRateCombined)
        fm.close()
    
main()
