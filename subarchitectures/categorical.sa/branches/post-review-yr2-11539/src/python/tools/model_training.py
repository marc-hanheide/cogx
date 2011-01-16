# Model training classes for Categorical.SA
# Author: Andrzej Pronobis

import tempfile
import shutil
import subprocess
import logging
import os.path

# ========================================
class Paths():
    
    def __init__(self, modelName, leaveTempFiles):
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
        # Setup the temporary dir
        self.tempDir = tempfile.mkdtemp('_categorical.sa_'+modelName)

    # ------------------------------------
    def runBin(self, cmd, args, successString):
        cmd = [self.binDir+'/'+cmd] + args
        self.logger.debug("------- %s -------", " ".join(cmd))
        out=subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).communicate()[0]
        out=out.strip()
        self.logger.debug(out)
        self.logger.debug("------- %s -------", " ".join(cmd))
        if len(successString)>0:
            if out.find(successString)<0:
                raise Exception("Error while running %s\n%s" % (" ".join(cmd), out))
        else:
            if len(out.strip())>0:
                raise Exception("Error while running %s\n%s" % (" ".join(cmd), out))
        return out

    # ------------------------------------
    def cleanTempDir(self):
        if not self.leaveTempFiles:
            self.logger.info("-> Cleaning temporary files...")
            shutil.rmtree(self.tempDir)
            self.logger.info("-> Done!")

    # ------------------------------------
    def getConfigFile(self, fileName):
        return self.configDir + "/" + fileName

    # ------------------------------------
    def getTempFile(self, fileName):
        return self.tempDir + "/" + fileName

    # ------------------------------------
    def getModelFile(self, fileName):
        return self.modelDir + "/" + fileName


# ========================================
paths = None
def initPaths(modelName, leaveTempFiles):
    global paths
    paths = Paths(modelName, leaveTempFiles)


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
        self.logger.info("-> Extracting geometrical features from the dataset %s..." % os.path.basename(self.dataSet))
        try:
            paths.runBin("laserextractor", 
                         [paths.getConfigFile("laser_features.cfg"), self.dataSet, self.outputFile],
                         "Closing files... Done!")
        except Exception as e:
            raise Exception("Error during geometrical feature extraction!\n" + str(e))
        self.logger.info("-> Done!")


# ========================================
class FeatureRescaler():
    # ------------------------------------
    def __init__(self):
        self.logger = logging.getLogger()

    # ------------------------------------
    def learnRanges(self, featureSets, rangeFile):
        self.logger.info("-> Learning scaling ranges...")
        self.mergedFeatures = paths.getTempFile("merged.features")
        try:
            # Merging datasets
            outFile = open(self.mergedFeatures, 'w')
            for f in featureSets:
                inFile = open(f, 'r')
                for l in inFile:
                    outFile.write(l)
                inFile.close()
            # Scaling
            paths.runBin("svm-scale", 
                         ['-s', rangeFile, self.mergedFeatures, '>/dev/null'],
                         "")
        except Exception as e:
            raise Exception("Error during scaling range learning!\n" + str(e))
        finally:
            inFile.close()
            outFile.close()
        self.logger.info("-> Done!")
    
    # ------------------------------------
    def rescaleSet(self, featureSet, scaledFeatureSet):
        self.logger.info("-> Rescaling feature set %s..." % featureSet)
        self.logger.info("-> Done!")


# ========================================
class Trainer():

    # ------------------------------------
    def __init__(self):
        self.logger = logging.getLogger()

    # ------------------------------------
    def trainFull(self, dataSets, gammas):
        self.dataSets = dataSets
        self.gammas = gammas
        try:
            try:
                self.logger.info("-> Starting the training procedure.")
                self._runFullTraining()
                self.logger.info("-> Training procedure completed successfully!")
            finally:
                paths.cleanTempDir()
        except Exception as e:
            self.logger.info("")
            self.logger.error(e)



# ========================================
class ShapeTrainer(Trainer):
    
    def __init__(self):
        Trainer.__init__(self)

    def _runFullTraining(self):
        # Feature extraction
        i=1
        self.featureSets=[]
        for d in self.dataSets:
            fSet = paths.getTempFile("dataset%d-shape.laser" % i)
            self.featureSets.append(fSet)
            gfe = GeometricalFeatureExtractor(d, fSet)
            gfe.extract()
            i=i+1
        # Rescaling
        self.rangeFile = paths.getModelFile("shape.lscl")
        fr = FeatureRescaler()
        fr.learnRanges(self.featureSets, self.rangeFile)
#        fr.learnRanges(self.featureFiles)
 #       i=1
  #      for d in self.dataSets:
        


