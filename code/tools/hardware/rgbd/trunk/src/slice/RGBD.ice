/**
 * Module to define data structures for RGB-D sensors
 * 
 */

#ifndef RGBD_ICE
#define RGBD_ICE

#include <cast/slice/CDL.ice>

module RGBD {

  // Depth from Xn is an array of uint16
  sequence<int> IntSeq;
  sequence<byte> ByteSeq;
  // The main data structure for Kinect Sensor data
  // TODO: add image, more metadata fields.
  struct KinectData{
    int XRes;
    int YRes;
    IntSeq depth;
    ByteSeq image;
    int frameid; 
  };

  // Kinect push client base class
  interface KinectPushClient{
    void receiveKinectData(KinectData data);
  };

  // All RGB-D push servers for different sensors.
  interface RGBDPushServer extends cast::interfaces::CASTComponent{
  
    void registerKinectPushClient(KinectPushClient* client, double interval);
  };

};

#endif // RGBD_ICE