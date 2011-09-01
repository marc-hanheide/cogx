Requirements
============
OpenCV (minimum 2.1)
OpenNI with Kinect sensor driver


kinectDemo
==========
Read a image and a color point cloud from Kinect sensor and show color image.
Grab data by pressing 'g' and quit program by pressing 'q'.


grabStereoKinectSequence
========================
Grab a sequence of images and point cloud data from a stereo setup, connected
via firewire and store it on the disk.


Compiling and running programs
==============================
 $ cmake .
 $ make
 $ ./kinectDemo
 $ ./grabStereoKinectSequence


------------------
Andreas Richtsfeld
Vienna University of Technology
Automation and Control Institute
ari@acin.tuwien.ac.at
skype: andreas.richtsfeld
