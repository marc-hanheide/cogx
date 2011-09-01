grabStereoKinectSequence
########################

Capture images and point cloud from a stereo setup and a kinect and store it in greylevel .bmp
or color .jpg format. The point clouds will be stored in .pts files.
FileReaderWriter.h provides a simple function to write and read this kind files.

There are two simple programs:
1. grabKinectSequence ... Grab a image with the point cloud from the Kinect.
2. grabStereoKinectSequence ... Grab images from a stereo setup and image and point cloud from the Kinect.

For both programs use the following buttons:
q ... Quit
g ... Grab grey-level images
c ... Grab color images

Dependencies:
- OpenNI
- OpenCV


For questions, please ask:

------------------
Andreas Richtsfeld
Automation and Control Institute
Vienna University of Technology
ari@acin.tuwien.ac.at

