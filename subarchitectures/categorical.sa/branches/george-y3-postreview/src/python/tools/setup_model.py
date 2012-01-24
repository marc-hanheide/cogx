#! /usr/bin/env python

import argparse
import os.path
import sys
from model_training import Paths


configFileContents = \
"# Config file for the model '%s'\n\
# Author: Andrzej Pronobis\n\
# Date: 2011-01-18\n\
\n\
[DataProvider]\n\
# Determines which data to grab\n\
UseLaser = True\n\
UseVision = True\n\
UseOdometry = True\n\
# Size of the scan/odometry queue in seconds and no. of elements\n\
QueueTimeWindow = 5.0\n\
QueueSize = 100\n\
# Index of the camera from which we grab images\n\
CameraIndex = 0\n\
# Delay between image pulling and scan lookup\n\
ScanDelay=150000\n\
\n\
[DataSaver]\n\
# Delay in us before new DP_CMD_UPDATE is sent.\n\
UpdateDelay=20000\n\
\n\
[PeekabotClient]\n\
RobotName = robot\n\
ConnectionRetryDelay = 1000000\n\
Port = 5050\n\
Hostname = localhost\n\
LoadScene = False\n\
Scene = Robone.xml\n\
\n\
[VisualProcessor]\n\
LabelFile = subarchitectures/categorical.sa/models/%s/appearance.lbl\n\
# CRFH settings\n\
CrfhDescriptors = Lxx(4,28)+Lxy(4,28)+Lyy(4,28)+Lxx(64,28)+Lxy(64,28)+Lyy(64,28)\n\
CrfhSmallValueFiltering = False\n\
CrfhFilteringThreshold = 0.0001\n\
CrfhSkipBorderPixels = 15\n\
# SVM\n\
ModelFile = subarchitectures/categorical.sa/models/%s/appearance.cmdl\n\
SvmOaoAlg = 1\n\
SvmOaaAlg = 1\n\
SvmMeasure = 1\n\
# Confidence estimation\n\
ConfidenceThreshold = 0.1\n\
# Delay, to control the frame rate\n\
Delay = 0\n\
\n\
[LaserProcessor]\n\
LabelFile = subarchitectures/categorical.sa/models/%s/shape.lbl\n\
# Feature extractor settings\n\
FeatureConfigFile = subarchitectures/categorical.sa/config/laser_features.cfg\n\
ScaleConfigFile = subarchitectures/categorical.sa/models/%s/shape.lscl\n\
# SVM\n\
ModelFile = subarchitectures/categorical.sa/models/%s/shape.lmdl\n\
SvmOaoAlg = 1\n\
SvmOaaAlg = 1\n\
SvmMeasure = 1\n\
# Confidence estimation\n\
ConfidenceThreshold = 0.1\n\
\n\
[AppearanceIntegrator]\n\
# Accumulation\n\
PositionBinSize = 0.2\n\
HeadingBinSize = 0.1\n\
# Max. influence of prior (in cache bins)\n\
MaxPriorOutputsCount = 20\n\
# No. of traveled cache bins after which pose information is forgotten (<0 means \"never\").\n\
CachePoseDecay = 30\n\
\n\
[ShapeIntegrator]\n\
# Accumulation\n\
PositionBinSize = 0.2\n\
HeadingBinSize = 0.1\n\
# Max. influence of prior (in cache bins)\n\
MaxPriorOutputsCount = 20\n\
# No. of traveled cache bins after which pose information is forgotten (<0 means \"never\").\n\
CachePoseDecay = 30\n"


includeFileContents = \
"# Include file for model '%s'\n\
\n\
SUBARCHITECTURE categorical.sa\n\
JAVA WM cast.architecture.SubarchitectureWorkingMemory\n\
CPP TM AlwaysPositiveTaskManager\n\
\n\
CPP GD categorical.dataprovider CategoricalDataProvider --config subarchitectures/categorical.sa/config/%s-shape-appearance.cfg\n\
CPP GD categorical.visualprocessor CategoricalVisualProcessor --config subarchitectures/categorical.sa/config/%s-shape-appearance.cfg\n\
CPP GD categorical.appearanceintegrator CategoricalAppearanceIntegrator --config subarchitectures/categorical.sa/config/%s-shape-appearance.cfg --placemanager place.manager\n\
CPP GD categorical.laserprocessor CategoricalLaserProcessor --config subarchitectures/categorical.sa/config/%s-shape-appearance.cfg\n\
CPP GD categorical.shapeintegrator CategoricalShapeIntegrator --config subarchitectures/categorical.sa/config/%s-shape-appearance.cfg --placemanager place.manager\n"



# =======================================
def parseArgs(title):
    parser = argparse.ArgumentParser(description=title)
    parser.add_argument('model_name',
                        help='name of the model')
    args = parser.parse_args()

    if args.model_name.find('/')>=0 or args.model_name.find('.')>=0:
        parser.error("Incorrect model name.")

    return args



# =======================================
def main():

    # Title & Args
    title = "Setting up models models for the Categorical Subarchitecture"
    print "".center(len(title)+4,'-')
    print "| %s |" % title
    print "".center(len(title)+4,'-')
    print ""

    args=parseArgs(title)
 
    # Initialize paths
    paths = Paths(args.model_name)

    # Create the config file
    configFileName = "%s-shape-appearance.cfg"  % args.model_name
    print "-> Creating the config file %s..." % configFileName 

    configFilePath = paths.getConfigFile(configFileName)
    tmp = configFileContents % (args.model_name,args.model_name,args.model_name,args.model_name,args.model_name,args.model_name)
    try:
        cf = open(configFilePath, 'w')
        cf.write(tmp)
        cf.close()
    except Exception as e:
        raise Exception("Cannot create the config file %s\n%s" % (configFilePath, str(e)))
    print "-> Done!"
    
    # Create the include file
    includeFileName = "categorical-%s-shape-appearance.cast" % args.model_name
    print "-> Creating the include file %s..." % includeFileName 

    includeFilePath = paths.getIncludesFile(includeFileName)
    tmp = includeFileContents % (args.model_name,args.model_name,args.model_name,args.model_name,args.model_name,args.model_name)
    try:
        inf = open(includeFilePath, 'w')
        inf.write(tmp)
        inf.close()
    except Exception as e:
        raise Exception("Cannot create the include file %s\n%s" % (includeFilePath, str(e)))
    print "-> Done!"
    print ""
    print "In order to use the model include the following line into your cast file:"
    print "INCLUDE includes/categorical.sa/" + includeFileName
    print

# =======================================
if __name__ == '__main__':
    main()