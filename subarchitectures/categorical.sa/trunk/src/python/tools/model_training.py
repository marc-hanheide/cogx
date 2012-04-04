# Model training classes for Categorical.SA
# Author: Andrzej Pronobis

import tempfile
import shutil
import subprocess
import logging
import os.path
import re
import glob
import sys
from copy import copy


# ========================================
# Returns a substring that matches the pattern. If the groupName is given, 
# only the named group can be returned, e.g.
# getMatchedSubStr("_N100_M", r'_N(?P<str>\d*)_M', "str")
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


# ========================================
def getDirFiles(path=".", mask="*"):
    list=glob.glob(os.path.join(os.path.expanduser(path),mask))
    list2=[]
    for l in list:
        if os.path.isfile(l):
            list2.append(os.path.basename(l))
    list2.sort()
    return list2


# ========================================
stepCount = 1.0 # Number of all steps
stepNo = 0.0 # Step number
def startStep():
    global stepCount
    global stepNo
    return "-> [%d%%] " % ((stepNo/stepCount) * 100.0)
def endStep():
    global stepCount
    global stepNo
    stepNo=stepNo+1
    return "-> [%d%%] " % ((stepNo/stepCount) * 100.0)


# ========================================
class Paths():
    # ------------------------------------
    def __init__(self, modelName, leaveTempFiles = False):
        self.tempDir=""
        # Logger
        self.logger = logging.getLogger()
        # Paths
        self.modelName = modelName
        self.leaveTempFiles = leaveTempFiles
        filePath = globals()['__file__']
        index = filePath.rfind('/subarchitectures/')
        if index<0:
            index = filePath.rfind('/output/')
        self.cogxRoot = filePath[0:index]
        self.binDir = self.cogxRoot + "/output/bin"
        self.configDir = self.cogxRoot + "/subarchitectures/categorical.sa/config"
        self.modelDir = self.cogxRoot + "/subarchitectures/categorical.sa/models/" + modelName
        self.includesDir = self.cogxRoot + "/instantiations/includes/categorical.sa"

    # ------------------------------------
    def createPaths(self):
        # Setup the temporary dir
        self.tempDir = tempfile.mkdtemp('_categorical.sa_'+self.modelName)
        # Setup the model dir
        if os.path.exists(self.modelDir):
            if not os.path.isdir(self.modelDir):
                raise Exception('%s exists and is not a directory!' % self.modelDir)
        else:
            os.makedirs(self.modelDir)

    # ------------------------------------
    def runBin(self, cmd, args, successString):
        cmd = self.binDir+'/'+cmd + " " + " ".join(args)
        
        self.logger.debug("\n------- %s -------", cmd)
        out=subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()[0]
        out=out.strip()
        self.logger.debug(out)
        self.logger.debug("------- %s -------\n", cmd)
        if len(successString)>0:
            if out.find(successString)<0:
                raise Exception("Error while running %s\n%s" % (cmd, out))
        else:
            if len(out.strip())>0:
                raise Exception("Error while running %s\n%s" % (cmd, out))
        return out

    # ------------------------------------
    def cleanTempDir(self):
        if not self.leaveTempFiles:
            self.logger.info(startStep()+"Cleaning temporary files...")
            if os.path.exists(self.tempDir):
                if os.path.isdir(self.tempDir):
                    shutil.rmtree(self.tempDir)
            self.logger.info(endStep()+"Done!")

    # ------------------------------------
    def getConfigFile(self, fileName):
        return self.configDir + "/" + fileName

    # ------------------------------------
    def getIncludesFile(self, fileName):
        return self.includesDir + "/" + fileName

    # ------------------------------------
    def getTempFile(self, fileName):
        return self.tempDir + "/" + fileName

    # ------------------------------------
    def getModelFile(self, fileName):
        return self.modelDir + "/" + fileName


# ========================================
paths = None
def initPaths(modelName, leaveTempFiles):
    logger = logging.getLogger()
    global paths
    try:
        paths = Paths(modelName, leaveTempFiles)
        paths.createPaths()
    except Exception as e:
        logger.info("")
        logger.error("Error while initializing paths!\n"+str(e))        
        sys.exit()


# ========================================
class FeatureExtractor():
    # ------------------------------------
    def __init__(self, dataSet, outputFile):
        self.dataSet = dataSet
        self.outputFile = outputFile
        self.logger = logging.getLogger()

    # ------------------------------------
    def extract(self):
        self._runExtraction()
        
        
# ========================================
class GeometricalFeatureExtractor(FeatureExtractor):
    # ------------------------------------
    def __init__(self, dataSet, outputFile):
        FeatureExtractor.__init__(self, dataSet, outputFile)
        
    # ------------------------------------
    def _runExtraction(self):
        self.logger.info(startStep()+"Extracting geometrical features from the dataset %s..." % os.path.basename(self.dataSet))
        try:
            paths.runBin("laserextractor", 
                         [paths.getConfigFile("laser_features.cfg"), self.dataSet, self.outputFile],
                         "Closing files... Done!")
        except Exception as e:
            raise Exception("Error during geometrical feature extraction from the dataset %s!\n%s" % 
                            (self.dataSet, str(e)))
        self.logger.info(endStep()+"Done!")


# ========================================
class CrfhFeatureExtractor(FeatureExtractor):
    # ------------------------------------
    def __init__(self, dataSet, outputFile):
        FeatureExtractor.__init__(self, dataSet, outputFile)
        
    # ------------------------------------
    def _runExtraction(self):
        self.logger.info(startStep()+"Preparing list of images for the dataset %s..." % os.path.basename(self.dataSet))
        try:
            try:
                cf = open(self.dataSet)
                cfLines = cf.readlines()
                cf.close()
            finally:
                cf.close()
            imageDir = os.path.dirname(self.dataSet)+"/"+getMatchedSubStr(cfLines, r'DataDir = (?P<str>.*)$', "str")
            filesList = getDirFiles(imageDir, "*.pgm")
            targetFile = os.path.dirname(self.dataSet)+"/"+getMatchedSubStr(cfLines, r'DataFile = (?P<str>.*)$', "str", 3)
            try:
                tf = open(targetFile)
                tfLines = tf.readlines()
                tf.close()
            finally:
                tf.close()
            imageList = paths.getTempFile("tmp-crfh.list")
            try:
                imf = open(imageList, 'w')
                if len(filesList) != len(tfLines):
                    raise Exception('The target file does not match the image files.')
                for i in range(len(filesList)):
                    if int(filesList[i].split('_')[0]) != int(tfLines[i].split(' ')[0]):
                        raise Exception('The target file does not match the image files.')
                    imf.write("%s/%s %s\n" % (imageDir, filesList[i], tfLines[i].split(' ')[1]))
                imf.close()
            finally:
                tf.close()
        except Exception as e:
            raise Exception("Error while preparing list of images for the dataset %s!\n%s" % 
                            (self.dataSet, str(e)))
        self.logger.info(endStep()+"Done!")
        self.logger.info(startStep()+"Extracting CRFH features from the dataset %s..." % os.path.basename(self.dataSet))
        try:
            paths.runBin("crfh-extract", 
                         ['"Lxx(4,28)+Lxy(4,28)+Lyy(4,28)+Lxx(64,28)+Lxy(64,28)+Lyy(64,28)"', "@"+imageList, self.outputFile],
                         "Finished!")
        except Exception as e:
            raise Exception("Error during CRFH feature extraction from the dataset %s!\n%s" % 
                            (self.dataSet, str(e)))
        self.logger.info(endStep()+"Done!")


# ========================================
class FeatureRescaler():
    # ------------------------------------
    def __init__(self):
        self.logger = logging.getLogger()

    # ------------------------------------
    def learnRanges(self, featureSet, rangeFile):
        self.logger.info(startStep()+"Learning scaling ranges from feature set %s..." % os.path.basename(featureSet))
        self.rangeFile = rangeFile
        try:
            # Scaling
            paths.runBin("svm-scale", 
                         ['-s', rangeFile, featureSet, '>/dev/null'],
                         "")
        except Exception as e:
            raise Exception("Error while scaling range learning from feature set %s!\n%s" % (featureSet, str(e)))
        self.logger.info(endStep()+"Done!")
    
    # ------------------------------------
    def rescaleSet(self, featureSet, scaledFeatureSet):
        self.logger.info(startStep()+"Rescaling feature set %s..." % os.path.basename(featureSet))
        try:
            paths.runBin("svm-scale", 
                         ['-r', self.rangeFile, featureSet, '>', scaledFeatureSet],
                         "")
        except Exception as e:
            raise Exception("Error while rescaling feature set %s!\n%s" % (featureSet, str(e)))
        self.logger.info(endStep()+"Done!")


# ========================================
class SetMerger():
    # ------------------------------------
    def __init__(self):
        self.logger = logging.getLogger()

    # ------------------------------------
    def mergeSets(self, featureSets, outputSet):
        baseFeatureSetNames=[]
        for f in featureSets:
            baseFeatureSetNames.append(os.path.basename(f))
        setNames = ', '.join(baseFeatureSetNames)
        self.logger.info(startStep()+"Merging feature sets %s into %s..." % (setNames, os.path.basename(outputSet)))
        try:
            outFile = open(outputSet, 'w')
            for f in featureSets:
                self.logger.debug("---> Adding %s" % f)
                try:
                    inFile = open(f, 'r')
                    for l in inFile:
                        outFile.write(l)
                    inFile.close()
                finally:
                    inFile.close()
        except Exception as e:
            raise Exception("Error while merging feature sets %s into %s!\n%s" %(setNames, os.path.basename(outputSet),str(e)))
        finally:
            outFile.close()
        self.logger.info(endStep()+"Done!")


# ========================================
class SvmModel:
    # ------------------------------------
    def __init__(self, modelFile):
        self.logger = logging.getLogger()
        self.modelFile = modelFile

    # ------------------------------------
    def train(self, trainingSet, kernel, gamma):
        self.logger.info(startStep()+"Training model %s from feature set %s using gamma=%s..." % 
                         (os.path.basename(self.modelFile), os.path.basename(trainingSet), gamma))
        try:
            paths.runBin("svm-train", ["-s 5", "-t", kernel, "-g", gamma, "-c 100", 
                         trainingSet, self.modelFile],
                         "-- Training time:")
        except Exception as e:
            raise Exception("Error while training model %s from feature set %s!\n%s" % (self.modelFile, trainingSet, str(e)))
        self.logger.info(endStep()+"Done!")
    
    # ------------------------------------
    def test(self, testingSet, outputFile):
        self.logger.info(startStep()+"Testing model %s on feature set %s..." % 
                         (os.path.basename(self.modelFile), os.path.basename(testingSet)))
        try:
            out = paths.runBin("svm-predict", [testingSet, self.modelFile, outputFile],
                         "Accuracy =")
            accuracy = getMatchedSubStr(out, r'Accuracy = (?P<str>[\d\.]*)%', "str")
        except Exception as e:
            raise Exception("Error while testing model %s on feature set %s!\n%s" % (self.modelFile, testingSet, str(e)))
        self.logger.info(endStep()+"Done (%s%%)!" % accuracy)
        return float(accuracy)
    

# ========================================
class Trainer():
    # ------------------------------------
    def __init__(self, gammas):
        self.logger = logging.getLogger()
        if len(gammas)>0:
            self.gammas = gammas
            self.bestGamma = gammas[0]
        else:
            self.gammas = self.defaultGammas
            self.bestGamma = self.defaultBestGamma

    # ------------------------------------
    def train(self, full, reduced, noParams, dataSets):
        self.dataSets = dataSets
        try:
            try:
                self.logger.info("-> Starting the training procedure.")
                if noParams:
                    self._runNoParamsTraining()
                elif reduced:
                    self._runReducedTraining()
                elif full:
                    self._runFullTraining()
                self.logger.info("-> Training procedure completed successfully!")
            finally:
                paths.cleanTempDir()
        except Exception as e:
            self.logger.info("")
            self.logger.error(e)
            sys.exit()


# ========================================
class ShapeTrainer(Trainer):
    
    defaultGammas=["0.0001", "0.000133352", "0.000177828", "0.000237137", "0.000316228", "0.000421697", "0.000562341", "0.000749894",
                   "0.001", "0.00133352", "0.00177828", "0.00237137", "0.00316228", "0.00421697", "0.00562341", "0.00749894",
                   "0.01", "0.0133352", "0.0177828", "0.0237137", "0.0316228", "0.0421697", "0.0562341", "0.0749894",
                   "0.1", "0.133352", "0.177828", "0.237137", "0.316228", "0.421697", "0.562341", "0.749894",
                   "1",   "1.33352",  "1.77828",  "2.37137",  "3.16228",  "4.21697",  "5.62341",  "7.49894",
                   "10",  "13.3352",  "17.7828",  "23.7137",  "31.6228",  "42.1697",  "56.2341",  "74.9894",
                   "100" ]
    
    defaultBestGamma = "0.07"


    # ------------------------------------
    def __init__(self, gammas):
        Trainer.__init__(self, gammas)

    # ------------------------------------
    def _runFullTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 2 + len(self.dataSets)*2 + len(self.gammas)*len(self.dataSets)*3 + 3
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets:
            fSet = paths.getTempFile("dataset%d-shape.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Merging sets for rescaling
        sm = SetMerger()
        self.mergedFeatures = paths.getTempFile("merged-shape.laser")
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        # Rescaling
        self.rangeFile = paths.getModelFile("shape.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.mergedFeatures, self.rangeFile)
        i=1
        self.scaledFeatureSets=[]
        for f in self.featureSets:
            sfSet = paths.getTempFile("dataset%d-shape.laser_scaled" % i)
            self.scaledFeatureSets.append(sfSet)
            fr.rescaleSet(f, sfSet)
            i=i+1
        # Parameter selection by cross-validation
        maxAccuracy = 0.0
        maxGamma = -1
        for g in self.gammas:
            overAllSetsAccuracy = 0.0
            for i in range(len(self.scaledFeatureSets)):
                # Create a training set for this partitioning
                tmp = copy(self.scaledFeatureSets)
                del tmp[i]
                sm.mergeSets(tmp, self.mergedFeatures)
                # Train
                tmpModelFile = paths.getTempFile("tmp-shape.lmdl")
                svm=SvmModel(tmpModelFile)
                svm.train(self.mergedFeatures, "2", g)
                # Test on the test set
                accuracy=svm.test(self.scaledFeatureSets[i], paths.getTempFile("tmp-shape.output"))
                overAllSetsAccuracy+=accuracy
            overAllSetsAccuracy/=float(len(self.scaledFeatureSets))
            if overAllSetsAccuracy > maxAccuracy:
                maxAccuracy = overAllSetsAccuracy
                maxGamma = g
        self.logger.info("-> Best recognition rate (over all sets) is %s for gamma=%s", str(maxAccuracy), maxGamma)
        # Final training
        sm.mergeSets(self.scaledFeatureSets, self.mergedFeatures)
        self.modelFile = paths.getModelFile("shape.lmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.mergedFeatures, "2", maxGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("shape.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: Full gamma cross-validation\n")
            inf.write("Features: geometrical features\n")
            inf.write("SVM Kernel: Gaussian\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % maxGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")


    # ------------------------------------
    def _runReducedTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 2 + 2*2 + len(self.gammas)*2 + 3
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets[:2]:
            fSet = paths.getTempFile("dataset%d-shape.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Merging sets for rescaling
        sm = SetMerger()
        self.mergedFeatures = paths.getTempFile("merged-shape.laser")
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        # Rescaling
        self.rangeFile = paths.getModelFile("shape.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.mergedFeatures, self.rangeFile)
        i=1
        self.scaledFeatureSets=[]
        for f in self.featureSets:
            sfSet = paths.getTempFile("dataset%d-shape.laser_scaled" % i)
            self.scaledFeatureSets.append(sfSet)
            fr.rescaleSet(f, sfSet)
            i=i+1
        # Parameter selection by cross-validation
        maxAccuracy = 0.0
        maxGamma = -1
        for g in self.gammas:
            # Train
            tmpModelFile = paths.getTempFile("tmp-shape.lmdl")
            svm=SvmModel(tmpModelFile)
            svm.train(self.scaledFeatureSets[0], "2", g)
            # Test on the test set
            accuracy=svm.test(self.scaledFeatureSets[1], paths.getTempFile("tmp-shape.output"))
            if accuracy > maxAccuracy:
                maxAccuracy = accuracy
                maxGamma = g
        self.logger.info("-> Best recognition rate is %s for gamma=%s", str(maxAccuracy), maxGamma)
        # Final training
        sm.mergeSets(self.scaledFeatureSets, self.mergedFeatures)
        self.modelFile = paths.getModelFile("shape.lmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.mergedFeatures, "2", maxGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("shape.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: Reduced gamma cross-validation\n")
            inf.write("Features: geometrical features\n")
            inf.write("SVM Kernel: Gaussian\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % maxGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")


    # ------------------------------------
    def _runNoParamsTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 2 + 2 + 2
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets[:2]:
            fSet = paths.getTempFile("dataset%d-shape.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Merging sets for rescaling
        sm = SetMerger()
        self.mergedFeatures = paths.getTempFile("merged-shape.laser")
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        # Rescaling
        self.rangeFile = paths.getModelFile("shape.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.mergedFeatures, self.rangeFile)
        i=1
        self.scaledFeatureSets=[]
        for f in self.featureSets:
            sfSet = paths.getTempFile("dataset%d-shape.laser_scaled" % i)
            self.scaledFeatureSets.append(sfSet)
            fr.rescaleSet(f, sfSet)
            i=i+1
        # Final training
        self.modelFile = paths.getModelFile("shape.lmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.scaledFeatureSets[0], "2", self.bestGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("shape.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: No parameter selection (default used)\n")
            inf.write("Features: geometrical features\n")
            inf.write("SVM Kernel: Gaussian\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % self.bestGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")



# ========================================
class SizeTrainer(Trainer):
    
    defaultGammas=["0.0001", "0.000133352", "0.000177828", "0.000237137", "0.000316228", "0.000421697", "0.000562341", "0.000749894",
                   "0.001", "0.00133352", "0.00177828", "0.00237137", "0.00316228", "0.00421697", "0.00562341", "0.00749894",
                   "0.01", "0.0133352", "0.0177828", "0.0237137", "0.0316228", "0.0421697", "0.0562341", "0.0749894",
                   "0.1", "0.133352", "0.177828", "0.237137", "0.316228", "0.421697", "0.562341", "0.749894",
                   "1",   "1.33352",  "1.77828",  "2.37137",  "3.16228",  "4.21697",  "5.62341",  "7.49894",
                   "10",  "13.3352",  "17.7828",  "23.7137",  "31.6228",  "42.1697",  "56.2341",  "74.9894",
                   "100" ]
    
    defaultBestGamma = "0.07"


    # ------------------------------------
    def __init__(self, gammas):
        Trainer.__init__(self, gammas)

    # ------------------------------------
    def _runFullTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 2 + len(self.dataSets)*2 + len(self.gammas)*len(self.dataSets)*3 + 3
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets:
            fSet = paths.getTempFile("dataset%d-size.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Merging sets for rescaling
        sm = SetMerger()
        self.mergedFeatures = paths.getTempFile("merged-size.laser")
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        # Rescaling
        self.rangeFile = paths.getModelFile("size.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.mergedFeatures, self.rangeFile)
        i=1
        self.scaledFeatureSets=[]
        for f in self.featureSets:
            sfSet = paths.getTempFile("dataset%d-size.laser_scaled" % i)
            self.scaledFeatureSets.append(sfSet)
            fr.rescaleSet(f, sfSet)
            i=i+1
        # Parameter selection by cross-validation
        maxAccuracy = 0.0
        maxGamma = -1
        for g in self.gammas:
            overAllSetsAccuracy = 0.0
            for i in range(len(self.scaledFeatureSets)):
                # Create a training set for this partitioning
                tmp = copy(self.scaledFeatureSets)
                del tmp[i]
                sm.mergeSets(tmp, self.mergedFeatures)
                # Train
                tmpModelFile = paths.getTempFile("tmp-size.lmdl")
                svm=SvmModel(tmpModelFile)
                svm.train(self.mergedFeatures, "2", g)
                # Test on the test set
                accuracy=svm.test(self.scaledFeatureSets[i], paths.getTempFile("tmp-size.output"))
                overAllSetsAccuracy+=accuracy
            overAllSetsAccuracy/=float(len(self.scaledFeatureSets))
            if overAllSetsAccuracy > maxAccuracy:
                maxAccuracy = overAllSetsAccuracy
                maxGamma = g
        self.logger.info("-> Best recognition rate (over all sets) is %s for gamma=%s", str(maxAccuracy), maxGamma)
        # Final training
        sm.mergeSets(self.scaledFeatureSets, self.mergedFeatures)
        self.modelFile = paths.getModelFile("size.lmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.mergedFeatures, "2", maxGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("size.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: Full gamma cross-validation\n")
            inf.write("Features: geometrical features\n")
            inf.write("SVM Kernel: Gaussian\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % maxGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")


    # ------------------------------------
    def _runReducedTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 2 + 2*2 + len(self.gammas)*2 + 3
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets[:2]:
            fSet = paths.getTempFile("dataset%d-size.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Merging sets for rescaling
        sm = SetMerger()
        self.mergedFeatures = paths.getTempFile("merged-size.laser")
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        # Rescaling
        self.rangeFile = paths.getModelFile("size.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.mergedFeatures, self.rangeFile)
        i=1
        self.scaledFeatureSets=[]
        for f in self.featureSets:
            sfSet = paths.getTempFile("dataset%d-size.laser_scaled" % i)
            self.scaledFeatureSets.append(sfSet)
            fr.rescaleSet(f, sfSet)
            i=i+1
        # Parameter selection by cross-validation
        maxAccuracy = 0.0
        maxGamma = -1
        for g in self.gammas:
            # Train
            tmpModelFile = paths.getTempFile("tmp-size.lmdl")
            svm=SvmModel(tmpModelFile)
            svm.train(self.scaledFeatureSets[0], "2", g)
            # Test on the test set
            accuracy=svm.test(self.scaledFeatureSets[1], paths.getTempFile("tmp-size.output"))
            if accuracy > maxAccuracy:
                maxAccuracy = accuracy
                maxGamma = g
        self.logger.info("-> Best recognition rate is %s for gamma=%s", str(maxAccuracy), maxGamma)
        # Final training
        sm.mergeSets(self.scaledFeatureSets, self.mergedFeatures)
        self.modelFile = paths.getModelFile("size.lmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.mergedFeatures, "2", maxGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("size.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: Reduced gamma cross-validation\n")
            inf.write("Features: geometrical features\n")
            inf.write("SVM Kernel: Gaussian\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % maxGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")


    # ------------------------------------
    def _runNoParamsTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 2 + 2 + 2
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets[:2]:
            fSet = paths.getTempFile("dataset%d-size.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Merging sets for rescaling
        sm = SetMerger()
        self.mergedFeatures = paths.getTempFile("merged-size.laser")
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        # Rescaling
        self.rangeFile = paths.getModelFile("size.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.mergedFeatures, self.rangeFile)
        i=1
        self.scaledFeatureSets=[]
        for f in self.featureSets:
            sfSet = paths.getTempFile("dataset%d-size.laser_scaled" % i)
            self.scaledFeatureSets.append(sfSet)
            fr.rescaleSet(f, sfSet)
            i=i+1
        # Final training
        self.modelFile = paths.getModelFile("size.lmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.scaledFeatureSets[0], "2", self.bestGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("size.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: No parameter selection (default used)\n")
            inf.write("Features: geometrical features\n")
            inf.write("SVM Kernel: Gaussian\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % self.bestGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")




# ========================================
class AppearanceTrainer(Trainer):
    
    defaultGammas=["0.001", "0.00133352", "0.00177828", "0.00237137", "0.00316228", "0.00421697", "0.00562341", "0.00749894",
                   "0.01", "0.0133352", "0.0177828", "0.0237137", "0.0316228", "0.0421697", "0.0562341", "0.0749894",
                   "0.1", "0.133352", "0.177828", "0.237137", "0.316228", "0.421697", "0.562341", "0.749894",
                   "1",   "1.33352",  "1.77828",  "2.37137",  "3.16228",  "4.21697",  "5.62341",  "7.49894",
                   "10",  "13.3352",  "17.7828",  "23.7137",  "31.6228" ]
    
    defaultBestGamma = "2.0"


    # ------------------------------------
    def __init__(self, gammas):
        Trainer.__init__(self, gammas)

    # ------------------------------------
    def _runFullTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 1 + len(self.dataSets)*2 + len(self.gammas)*len(self.dataSets)*3 + 3
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets:
            fSet = paths.getTempFile("dataset%d-appearance.crfh" % i)
            self.featureSets.append(fSet)
            cfe = CrfhFeatureExtractor(d, fSet)
            cfe.extract()
            i=i+1
        # Parameter selection by cross-validation
        self.mergedFeatures = paths.getTempFile("merged-appearance.crfh")
        sm = SetMerger()
        maxAccuracy = 0.0
        maxGamma = -1
        for g in self.gammas:
            overAllSetsAccuracy = 0.0
            for i in range(len(self.featureSets)):
                # Create a training set for this partitioning
                tmp = copy(self.featureSets)
                del tmp[i]
                sm.mergeSets(tmp, self.mergedFeatures)
                # Train
                tmpModelFile = paths.getTempFile("tmp-appearance.cmdl")
                svm=SvmModel(tmpModelFile)
                svm.train(self.mergedFeatures, "5", g)
                # Test on the test set
                accuracy=svm.test(self.featureSets[i], paths.getTempFile("tmp-appearance.output"))
                overAllSetsAccuracy+=accuracy
            overAllSetsAccuracy/=float(len(self.featureSets))
            if overAllSetsAccuracy > maxAccuracy:
                maxAccuracy = overAllSetsAccuracy
                maxGamma = g
        self.logger.info("-> Best recognition rate (over all sets) is %s for gamma=%s", str(maxAccuracy), maxGamma)
        # Final training
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        self.modelFile = paths.getModelFile("appearance.cmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.mergedFeatures, "5", maxGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("appearance.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: Full gamma cross-validation\n")
            inf.write("Features: CRFH Lxx(4,28)+Lxy(4,28)+Lyy(4,28)+Lxx(64,28)+Lxy(64,28)+Lyy(64,28)\n")
            inf.write("SVM Kernel: Chi2\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % maxGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")


    # ------------------------------------
    def _runReducedTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 1 + 2*2 + len(self.gammas)*2 + 3
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets[:2]:
            fSet = paths.getTempFile("dataset%d-appearance.crfh" % i)
            self.featureSets.append(fSet)
            cfe = CrfhFeatureExtractor(d, fSet)
            cfe.extract()
            i=i+1
        # Parameter selection by cross-validation
        self.mergedFeatures = paths.getTempFile("merged-appearance.crfh")
        sm = SetMerger()
        maxAccuracy = 0.0
        maxGamma = -1
        for g in self.gammas:
            # Train
            tmpModelFile = paths.getTempFile("tmp-appearance.cmdl")
            svm=SvmModel(tmpModelFile)
            svm.train(self.featureSets[0], "5", g)
            # Test on the test set
            accuracy=svm.test(self.featureSets[1], paths.getTempFile("tmp-appearance.output"))
            if accuracy > maxAccuracy:
                maxAccuracy = accuracy
                maxGamma = g
        self.logger.info("-> Best recognition rate is %s for gamma=%s", str(maxAccuracy), maxGamma)
        # Final training
        sm.mergeSets(self.featureSets, self.mergedFeatures)
        self.modelFile = paths.getModelFile("appearance.cmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.mergedFeatures, "5", maxGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("appearance.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: Reduced gamma cross-validation\n")
            inf.write("Features: CRFH Lxx(4,28)+Lxy(4,28)+Lyy(4,28)+Lxx(64,28)+Lxy(64,28)+Lyy(64,28)\n")
            inf.write("SVM Kernel: Chi2\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % maxGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")


    # ------------------------------------
    def _runNoParamsTraining(self):
        # Calculate step count
        global stepCount
        global paths
        stepCount = 1 + 2 + 2
        if not paths.leaveTempFiles:
            stepCount = stepCount+1
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets[:2]:
            fSet = paths.getTempFile("dataset%d-appearance.crfh" % i)
            self.featureSets.append(fSet)
            cfe = CrfhFeatureExtractor(d, fSet)
            cfe.extract()
            i=i+1
        # Final training
        self.mergedFeatures = paths.getTempFile("merged-appearance.crfh")
        sm = SetMerger()
        self.modelFile = paths.getModelFile("appearance.cmdl")
        svm = SvmModel(self.modelFile)
        svm.train(self.featureSets[0], "5", self.bestGamma)
        # Writing info file
        self.infoFile = paths.getModelFile("appearance.info")
        self.logger.info(startStep()+"Writing info file...")
        try:
            inf = open(self.infoFile, "w")
            inf.write("Training sets:\n")
            for d in self.dataSets:
                inf.write(" - %s\n" % d)
            inf.write("Parameter selection: No parameter selection (default used)\n")
            inf.write("Features: CRFH Lxx(4,28)+Lxy(4,28)+Lyy(4,28)+Lxx(64,28)+Lxy(64,28)+Lyy(64,28)\n")
            inf.write("SVM Kernel: Chi2\n")
            inf.write("SVM Multi-class: One-against-all\n")
            inf.write("SVM C: 100\n")
            inf.write("SVM Kernel Gamma: %s\n" % self.bestGamma)
        except Exception as e:
            raise Exception("Error while writing the info file %s!\n%s" % (self.infoFile, str(e)))
        finally:
            inf.close()
        self.logger.info(endStep()+"Done!")
