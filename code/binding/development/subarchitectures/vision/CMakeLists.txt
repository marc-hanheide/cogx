IF (NOT COSY_ROOT)

   # Set the CoSy root... MAKE SURE THIS POINTS TO THE code DIR
   GET_FILENAME_COMPONENT(COSY_ROOT ../.. ABSOLUTE)

   # Set the output directory.
   SET(OUTPUT ${COSY_ROOT}/output)

ENDIF (NOT COSY_ROOT)

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX ${OUTPUT} CACHE INTERNAL "")

# Set the path to additional CMake modules (e.g., our FindOmniORB module).
SET(CMAKE_MODULE_PATH ${COSY_ROOT}/tools/CMakeModules)

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Set c++ and idl root.
SET(CPP_ROOT ${CMAKE_SOURCE_DIR}/src/c++)
SET(IDL_ROOT ${CMAKE_SOURCE_DIR}/src/idl)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
INCLUDE_DIRECTORIES(${OUTPUT}/include/binding)
LINK_DIRECTORIES(${OUTPUT}/lib)

# You can prepare verbose Makefile to see what's going.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Find OmniORB package.
FIND_PACKAGE(OmniORB)


# Specify the list of subdirectories to process.

ADD_SUBDIRECTORY(src/c++/vision)
ADD_SUBDIRECTORY(src/c++/vision/idl)

OPTION(BUILD_VISION_COMPONENTS "Build vision components" YES)

IF (BUILD_VISION_COMPONENTS)

OPTION(BUILD_VISUAL_WORKING_MEMORY "Build visual working memory" NO)
OPTION(BUILD_VISION_GENERAL "Build general vision code" YES)

OPTION(BUILD_CAMERA_SERVER "Build camera server" YES)
OPTION(BUILD_VIDEO_SERVER "Build video server" NO)
#OPTION(BUILD_IMAGE_READER "Build image reader test" NO) # for testing 

OPTION(BUILD_VISION_TESTER "Build vision tester" NO)

#OPTION(BUILD_AFFORDANCE_RECORDER "Build the affordance recorder" NO)
#OPTION(BUILD_OBJECT_CLASS_DETECTOR "Build the object class detector" NO)
OPtion(BUILD_MULTI_OBJECT_CLASS_DETECTOR "Build the multi object class detector" NO)
OPTION(BUILD_VISION_BINDING_MONITOR "Build visual binding monitor (requires binding sa)" YES)
#OPTION(BUILD_SALIENCY "Build saliency based ROI detector" NO)
OPTION(BUILD_FEATURE_LEARNING "Build UOL feature learning component" NO)
OPTION(BUILD_FEATURE_LEARNING_CTF "Build Matlab CTF for UOL FL; off=use prebuilt CTF from SVN" NO)
MARK_AS_ADVANCED(BUILD_FEATURE_LEARNING_CTF)
#OPTION(BUILD_SURFACE_DETECTOR "Build the surface detector component" NO)
OPTION(BUILD_CHANGE_DETECTOR "Build the change detector" NO)
OPTION(BUILD_SEGMENTOR "Build the segmentor" NO)
#OPTION(BUILD_OBJECT_CLASS_DETECTOR "Build the object class detector" NO)
OPTION(BUILD_COLOR_COMPONENT "Build the color component" NO)
OPTION(BUILD_SHAPE_COMPONENT "Build the shape component" NO)
OPTION(BUILD_SIFT "Build the SIFT component" NO)
OPTION(BUILD_OBJECT_TRACKER "Build the object tracker" NO)
OPTION(BUILD_PERCEPT_GROUP "Build the Perceptual Grouping component" NO)
OPTION(BUILD_VISUALIZER "Build the visualizer" NO)

#OPTION(BUILD_NUMB_ARSE "Dumb component to test AffordaceRecorder" NO)
OPTION(BUILD_HAND_POINTER "Build the hand pointer" NO)

# Option for setting the planner system that plans sensing actions...
#OPTION(BUILD_VISUAL_PLANNER "Build the visual planner" NO)
OPTION(BUILD_CAR_CLASSIFIER "Build the car classfier" NO)

IF(BUILD_VISUAL_WORKING_MEMORY)
	ADD_SUBDIRECTORY(src/c++/vision/components/VisualWorkingMemory)
ENDIF(BUILD_VISUAL_WORKING_MEMORY)

IF (BUILD_VISION_GENERAL)
  ADD_SUBDIRECTORY(src/c++/vision/utils)
#  ADD_SUBDIRECTORY(src/c++/vision/components/common)	
ENDIF (BUILD_VISION_GENERAL)

IF(BUILD_CAR_CLASSIFIER)
  ADD_SUBDIRECTORY(src/c++/vision/components/CarClassifier)
ENDIF(BUILD_CAR_CLASSIFIER)

IF(BUILD_MULTI_OBJECT_CLASS_DETECTOR)
  ADD_SUBDIRECTORY(src/c++/vision/components/ObjectMClassDetector)
ENDIF(BUILD_MULTI_OBJECT_CLASS_DETECTOR)
IF(BUILD_CAMERA_SERVER)
  ADD_SUBDIRECTORY(src/c++/vision/components/CameraServer)
ENDIF(BUILD_CAMERA_SERVER)

IF(BUILD_VIDEO_SERVER)
  ADD_SUBDIRECTORY(src/c++/vision/components/VideoServer)
ENDIF(BUILD_VIDEO_SERVER)

#IF (BUILD_IMAGE_READER)
#  ADD_SUBDIRECTORY(src/c++/vision/components/ImageReader)
#ENDIF (BUILD_IMAGE_READER)

IF (BUILD_VISION_TESTER)
  ADD_SUBDIRECTORY(src/c++/vision/testing)
ENDIF (BUILD_VISION_TESTER)

IF (BUILD_FEATURE_LEARNING)
   IF (BUILD_FEATURE_LEARNING_CTF)
      ADD_SUBDIRECTORY(src/matlab)
   ELSE (BUILD_FEATURE_LEARNING_CTF)
      ADD_SUBDIRECTORY(prebuilt/featureLearning2)   
   ENDIF (BUILD_FEATURE_LEARNING_CTF)
   FIND_FILE( # TODO: Why is FIND_LIBRARY (FeatureLearningProxy) not working????
      FEATURE_LEARNING_MATLAB_PROXY
      NAMES libFeatureLearningProxy.so
      PATHS ${OUTPUT}  ${OUTPUT}/lib
      NO_DEFAULT_PATH
      DOC "Matlab Library required for FeatureLearning2"
   )
   IF (FEATURE_LEARNING_MATLAB_PROXY)
      ADD_SUBDIRECTORY(src/c++/vision/components/featurelearning2)
   ELSE(FEATURE_LEARNING_MATLAB_PROXY)
      MESSAGE(
         "FeatureLearningProxy library not installed, yet.\n"
         "   Run 'make install' to generate and install it.\n"
         "   Then run 'ccmake' again.\n"
         "   (For now, FeatureLearning won't be built)\n"
         "   (see also: BUILD_FEATURE_LEARNING_CTF)"
      )
      ENDIF (FEATURE_LEARNING_MATLAB_PROXY)
ENDIF (BUILD_FEATURE_LEARNING)

#IF (BUILD_SALIENCY)
#  FIND_PACKAGE(VXL)
#  IF(VXL_FOUND)
#    INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
#    ADD_SUBDIRECTORY(src/c++/vision/components/SalientRoiDetector)
#  ELSE(VXL_FOUND)
#    MESSAGE("VXL_DIR should be set to the VXL build directory.")
#    MESSAGE("VXL_DIR must be set in order to build SalientRoiDetector.")
#  ENDIF(VXL_FOUND)
#ENDIF (BUILD_SALIENCY)


IF (BUILD_CHANGE_DETECTOR)
   ADD_SUBDIRECTORY(src/c++/vision/components/ChangeDetector)
ENDIF (BUILD_CHANGE_DETECTOR)

IF (BUILD_SEGMENTOR)
  ADD_SUBDIRECTORY(src/c++/vision/components/Segmentor)
ENDIF (BUILD_SEGMENTOR)

IF (BUILD_VISUALIZER)
  ADD_SUBDIRECTORY(src/c++/vision/components/Visualizer)
ENDIF (BUILD_VISUALIZER)



#IF (BUILD_OBJECT_CLASS_DETECTOR)
#  ADD_SUBDIRECTORY(src/c++/vision/components/ObjectClassDetector)
#ENDIF (BUILD_OBJECT_CLASS_DETECTOR)

IF (BUILD_COLOR_COMPONENT)
   ADD_SUBDIRECTORY(src/c++/vision/components/ColorComponent)
ENDIF (BUILD_COLOR_COMPONENT)

IF (BUILD_SHAPE_COMPONENT)
   ADD_SUBDIRECTORY(src/c++/vision/components/ShapeComponent)
ENDIF (BUILD_SHAPE_COMPONENT)

IF (BUILD_SIFT)
  FIND_PACKAGE(VXL)
  IF(VXL_FOUND)
    INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
    ADD_SUBDIRECTORY(src/c++/vision/components/SIFTComponent)
  ELSE(VXL_FOUND)
    MESSAGE("VXL_DIR should be set to the VXL build directory.")
    MESSAGE("VXL_DIR must be set in order to build SIFTComponent.")
  ENDIF(VXL_FOUND)
ENDIF (BUILD_SIFT)

IF (BUILD_PERCEPT_GROUP)
    ADD_SUBDIRECTORY(src/c++/vision/components/PerceptualGrouping)
ENDIF (BUILD_PERCEPT_GROUP)

#IF (BUILD_SURFACE_DETECTOR)
#  ADD_SUBDIRECTORY(src/c++/vision/components/SurfaceDetector)
#  ADD_SUBDIRECTORY(src/c++/vision/components/DemoGoalTrigger)
#ENDIF (BUILD_SURFACE_DETECTOR)

IF (BUILD_VISION_BINDING_MONITOR)
   FIND_PACKAGE(Boost)	
   INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
   LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
   ADD_SUBDIRECTORY(src/c++/vision/components/VisionBindingMonitor)
#   ADD_SUBDIRECTORY(src/c++/vision/components/TrackedObjectMonitor)
#   ADD_SUBDIRECTORY(src/c++/vision/components/VisionComparator)
ENDIF (BUILD_VISION_BINDING_MONITOR)

IF (BUILD_OBJECT_TRACKER)
   ADD_SUBDIRECTORY(src/c++/vision/components/common/SystemUtils)
   ADD_SUBDIRECTORY(src/c++/vision/components/common/GeomUtils)
   ADD_SUBDIRECTORY(src/c++/vision/components/common/VisualizationUtils)
   ADD_SUBDIRECTORY(src/c++/vision/components/ObjectTracker)
ENDIF (BUILD_OBJECT_TRACKER)

#IF (BUILD_AFFORDANCE_RECORDER)
#  ADD_SUBDIRECTORY(src/c++/vision/components/AffordanceRecorder)
#ENDIF (BUILD_AFFORDANCE_RECORDER)

IF (BUILD_OBJECT_CLASS_DETECTOR)
  ADD_SUBDIRECTORY(src/c++/vision/components/ObjectClassDetector)
ENDIF (BUILD_OBJECT_CLASS_DETECTOR)

#IF (BUILD_MULTI_OBJECT_CLASS_DETECTOR)
#  ADD_SUBDIRECTORY(src/c++/vision/components/MultiObjectClassDetector)
#ENDIF (BUILD_MULTI_OBJECT_CLASS_DETECTOR)
#IF (BUILD_NUMB_ARSE)
#  ADD_SUBDIRECTORY(src/c++/vision/components/NumbArse)
#ENDIF (BUILD_NUMB_ARSE)

IF (BUILD_HAND_POINTER)
  ADD_SUBDIRECTORY(src/c++/vision/components/HandPointer)
ENDIF (BUILD_HAND_POINTER)

#IF (BUILD_VISUAL_PLANNER)
#   ADD_SUBDIRECTORY(src/c++/vision/components/Planning/Trigger)
#   ADD_SUBDIRECTORY(src/c++/vision/components/Planning/SimpleStateGenerator)
#   ADD_SUBDIRECTORY(src/c++/vision/components/Planning/VisualStateGenerator)
#   ADD_SUBDIRECTORY(src/c++/vision/components/Planning/SimpleActionGenerator)
#   ADD_SUBDIRECTORY(src/c++/vision/components/Planning/VisualStateGenerator)
#ENDIF (BUILD_VISUAL_PLANNER)

ENDIF (BUILD_VISION_COMPONENTS)
