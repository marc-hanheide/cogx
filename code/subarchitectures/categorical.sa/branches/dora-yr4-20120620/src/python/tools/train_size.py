#! /usr/bin/env python

import argparse
import os.path
import sys
import logging.handlers
from model_training import initPaths, SizeTrainer

# =======================================
def printOptions(args, title):
    logger = logging.getLogger()
    logger.info("".center(len(title)+4,'-'))
    logger.info("| %s |" % title)
    logger.info("".center(len(title)+4,'-'))
    logger.info("")
    logger.info("Selected options:")
    logger.info(" - Model name: " + args.model_name)
    if args.no_params:
        logger.info(" - Procedure: No parameter selection")
    elif args.reduced:
        logger.info(" - Procedure: Reduced cross-validation")
    elif args.full:
        logger.info(" - Procedure: Full cross-validation")
    if args.gammas:
        logger.info(" - Gamma values: " + args.gammas)
    else:
        logger.info(" - Gamma values: Default ")
    i=1
    for d in args.dataset:
        logger.info(" - Dataset %i: %s" % (i, d))
        i=i+1
    if args.verbose:
        logger.info(" - Verbose mode: On")
    else:
        logger.info(" - Verbose mode: Off")
    if args.log:
        logger.info(" - Log file: " + args.log)
    else:
        logger.info(" - Log file: Off")
    if args.temp:
        logger.info(" - Temporary files: Leave")
    else:
        logger.info(" - Temporary files: Delete")
    logger.info("") 

  
# =======================================
def parseArgs(title):
    parser = argparse.ArgumentParser(description=title)
    parser.add_argument('model_name',
                        help='name of the model')
    parser.add_argument('dataset', nargs='+', 
                        help='path to a dataset')
    group1 = parser.add_mutually_exclusive_group(required=True)
    group1.add_argument('--no-params', action='store_true', default=False,
                        help='run the training procedure without parameter selection; only one dataset is required and gamma can be specified using --gamma')
    group1.add_argument('--reduced', action='store_true', default=False,
                        help='run the reduced cross-validation procedure; two datasets are required and the second one is always used for testing')
    group1.add_argument('--full', action='store_true', default=False,
                        help='run the full cross-validation procedure; at least two datasets are required')
    parser.add_argument('--gammas',  metavar='values', action='store', default="",
                       help='comma separated list of gamma values')
    group3 = parser.add_argument_group('debugging')
    group3.add_argument('--temp', action='store_true', default=False,
                        help='do not delete the temporary files')
    group3.add_argument('--verbose', action='store_true', default=False,
                        help='be verbose')
    group3.add_argument('--log', action='store_true', default=False,
                        help='create a log file')
    args = parser.parse_args()

    # Check arguments
    if args.no_params and len(args.dataset)!=1:
        parser.error("Only one dataset can be provided if the --no-params option is used.")
    elif args.reduced and len(args.dataset)!=2:
        parser.error("Two datasets must be provided if the --reduced option is used.")
    elif args.full and len(args.dataset)<2:
        parser.error("At least two datasets are required for the full training procedure.")

    if args.log:
        args.log=os.path.basename(sys.argv[0]).split('.')[0]+".log"

    if len(args.gammas)>0:
        args.gammas = args.gammas.split(',')
    else:
        args.gammas=[]

    if args.model_name.find('/')>=0 or args.model_name.find('.')>=0:
        parser.error("Incorrect model name.")

    return args


# =======================================
def configureLog(args):
    # Set up a specific logger with our desired output level
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    # Add the log message handlers to the logger
    if args.log:
        rollOver=False
        if os.path.exists(args.log):
            rollOver=True
        fileHandler = logging.handlers.RotatingFileHandler(args.log, backupCount=5)
        if rollOver:
            fileHandler.doRollover()
        fileHandler.setLevel(logging.DEBUG)
        logger.addHandler(fileHandler)
    streamHandler = logging.StreamHandler(sys.stdout)    
    if args.verbose:
        streamHandler.setLevel(logging.DEBUG)
    else:
        streamHandler.setLevel(logging.INFO)
    logger.addHandler(streamHandler)


# =======================================
def main():

    # Title
    propertyName="size"
    title = "Training '%s' property models for the Categorical Subarchitecture" % propertyName

    # Args
    args=parseArgs(title)
    configureLog(args)
    printOptions(args, title)
 
    # Initialize paths
    initPaths(args.model_name, args.temp)
 
    # Run training
    logging.getLogger().info("Performing training:")
    sizeTrainer = SizeTrainer(args.gammas)
    sizeTrainer.train(args.full, args.reduced, args.no_params, args.dataset)
        

# =======================================
if __name__ == '__main__':
    main()
