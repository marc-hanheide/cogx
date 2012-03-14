#ifndef VIDEO_ICE
#define VIDEO_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

module Video {

  sequence<byte> ByteSeq;
  sequence<int> IntSeq;

  // TODO: add iamge scale factor for properly dealing with subsampled images
  /**
   * Camera class.
   * For some theory:
   * Sonka, Hlavac, Boyle: Image Processing, Analysy and Machine Vision
   *   2nd ed., chapter 9.2 Geometry for 3D Vision, p. 448.
   *   Note however that we use positive entries in the camera calibration matrix
   *   (i.e. we place the image plane conveniently in front of the focal point).
   * Trucco, Verri: Introductory Techniques for 3D Computer Vision
   *   chapter 2.4 Camera parameters, p. 34
   *   Note again, we use positive entries.
   * Hartley, Zisserman: Multiple View Geometry, 2nd ed, 2003, Cambridge
   *   University Press
   * Intel OpenCV library manual, chapter 3D Reconstruction,
   *
   * We use the following notation:
   * Matrices (R, A) are upper case. Vectors (t, u) are lower case, homogenous
   * vectors (U) are upper case.
   *
   * We have the following coordinate systems:
   * world: 3D coordinate system, units are [m], origin and orientation arbitrary
   * camera: 3D coordinate system, units are [m], origin in camera center of
   *         projection, x points to the "right", y points "down" and z points
   *         "away", all w.r.t. the world coordinate system
   * image plane: 2D coordinate system, units are [m], origin at the intersection
   *              of principal ray and image plane, x points to the "right" and y
   *              points "down"
   * image pixel: 2D coordinate system, units are [pixel], origin is in upper left
   *              corner of the image, x points "right" and y points "down"
   *
   * w  = [w_x w_y w_z]  .. point in world coords
   * p  = [p_x p_y p_z]  .. point in camera coords
   * U  = [U_X U_Y U_W]  .. point in homogeneous image pixel coords
   * u  = [u_x u_y]      .. ideal point in image pixel coords
   * u' = [u_x' u_y']    .. distorted point in image pixel coords
   *
   * extrinsic parameters, pose:
   *   w = R*p + t   R .. rotation matrix, t .. translation vector
   *                 Note that R,t is the pose of the camera w.r.t. the world
   *                 origin. Some references use the inverse, i.e. p = R*w + t,
   *                 where R,t is now the pose of the world w.r.t. to the camera.
   * thus
   *   p = R^T*(w - t)
   *
   * projection to image:
   *   U = A*p       A .. 3x3 camera matrix
   *
   *       | f_x  0  c_x |   f_x = f/s_x where f .. focal length in mm
   *   A = |  0  f_y c_y |                     s_x .. pixel size in mm/pix
   *       |  0   0   1  |   c_x, c_y .. camera principal point in pix
   *
   *   u_x = U_X/U_W
   *   u_x = U_Y/U_W
   * i.e. written out:
   *   u_x = f_x*p_x/p_z + c_x
   *   u_y = f_y*p_y/p_z + c_y
   *
   * distortion:
   * with
   *   x = p_x/p_z
   *   y = p_y/p_z
   *   r^2 = x^2 + y^2
   *
   *   x' = x*(1 + k1*r^2 + k2*r^4) + 2*p1*x*y + p2*(r^2 + 2*x^2)
   *   y' = y*(1 + k1*r^2 + k2*r^4) + p1*(r2 + 2*y^2) + 2*p2*x*y
   *
   *   u_x' = f_x*x' + c_x
   *   u_y' = f_x*y' + c_y
   */
  struct CameraParameters {
    int id;				
    // image dimension
    int width;
    int height;
    // Instrinsic parameters:
    // entries of the camera matrix
    double fx;
    double fy;
    double cx;
    double cy;
    // radial distortion parameters
    double k1;
    double k2;
    double k3;
    // tangential distortion parameters
    double p1;
    double p2;
    // extrinsic parameters: 3D pose of camera w.r.t. world
    cogx::Math::Pose3 pose;
    // time stamp (important for cameras on pan-tilt head etc.)
    cast::cdl::CASTTime time;
  };

  /**
   * *sigh* Need this wrapper CLASS around CameraParameters STRUCT, cause
   * otherwise it just won't work bla bla bla
   * 
   * NOTE: CameraMount only fills pose and time. The video server will fill the
   * rest.
   */
  class CameraParametersWrapper {
    CameraParameters cam;
  };

  // Subscribe to CameraMotionState when you need to work with a stable camera.
  // The motion is detected for selected cameras by CameraMount which monitors
  // the PTZ server.
  class CameraMotionState {
    int id;
    bool bMoving;
  };

  /**
   * Image class for our system.
   * Image format is RGB24 (we don't want to fiddle around with dozens of
   * different image formats).
   * Apart from raw image data, each image also has a time stamp and the id of
   * the camera which took the image. These are required to associate the image
   * with other sensory data in the system.
   */
  struct Image {
    int width;
    int height;
    // data: 3*width*height bytes of data
    ByteSeq data;
    cast::cdl::CASTTime time;
    int camId;
    CameraParameters camPars;
  };

  sequence<Image> ImageSeq;

  /**
   * A video server, serving images from one or more cameras.
   */
  interface VideoInterface {
    int getNumCameras();
    void getImageSize(out int width, out int height);
    bool getCameraParameters(int camId, out CameraParameters camPars);
    int getFramerateMilliSeconds();
    void getImage(int camId, out Image img);
    void getImages(out ImageSeq images);
    //void getImages(IntSeq camIds, out ImageSeq images);
    //void getScaledImage(int camId, int width, int height, out Image img);
    void getScaledImages(int width, int height, out ImageSeq images);
    //void getScaledImages(IntSeq camIds, int width, int height, out ImageSeq images);
    bool getHRImages(out ImageSeq images);
    void startReceiveImages(string receiverComponentId, IntSeq camIds, int width, int height);
    void stopReceiveImages(string receiverComponentId);
    void changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int fps);
    bool inFormat7Mode();
    string getServerName();
  };
  
  interface VideoClientInterface {
    void receiveImages(ImageSeq images);
    void receiveImages2(string serverName, ImageSeq images);
  };

  // The structure contains parameters that describe the sequence of images
  // that should be loaded.
  class VideoSequenceInfo {
    // List of filename templates, one for each camera.
    // The list is space-delimited to be compatible with --files in configure().
    string fileTemplates;

    // Start index, end index, step.
    int start;
    int end;
    int step;

    // True, if the sequence should play continuously.
    bool loop;

    // How many times to repeat each frame.
    // Framerate should be set in configure().
    int repeatFrame;
  };
};

#endif

