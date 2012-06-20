CALIB STEREO
============

Calibration of a stereo setup (which might also be the left stereo and the kinect camera). Output is for distorted and undistorted images: 
	- camcalib-left.xml =>	  size, intrinsic, distortion, projection, rotation, tvec, rmat
	- camcalib-right.xml =>   size, intrinsic, distortion, projection, rotation, tvec, rmat
  and
	- campose-left.xml =>	  tvec, rmat
	- campose-right.xml =>    tvec, rmat

Compile:
	mkdir BUILD
	cd BUILD
	ccmake ..  (press c and g)
	make

Command:
	./calib_stereo -h 28 -w 28 -A ./img/left -B ./img/right/


For calibration of the Kinect and the stereo setup use first the left and right images to calibrate the stereo setup and then the left and kinect images to calibrate the kinect camera in respect to the left stereo camera. (rename the camcalib-right.xml => camcalib-kinect.xml / The left calibration file should be the same than for the stereo calibration.)

------------------
Andreas Richtsfeld
ari@acin.tuwien.ac.at
Vienna University of Technology




Example:
	./calib_stereo -h 28 -w 28 -A /home/ari/projects/saa-cogx/tuw/instantiations/11-05-11/all/left -B /home/ari/projects/saa-cogx/tuw/instantiations/11-05-11/all/right/


