/**
 * Module to define data structures for RGB-D sensors
 * 
 */

#ifndef RGBD_ICE
#define RGBD_ICE

#include <cast/slice/CDL.ice>

module RGBD {

sequence<int> DepthData;
struct KinectData{
int XRes;
int YRes;
DepthData depth;
int frameid;
};
interface KinectPushClient{
void receiveKinectData(KinectData data);
};

interface RGBDPushServer extends cast::interfaces::CASTComponent{
void registerKinectPushClient(KinectPushClient* client, double interval);
};


};


#endif // LASER_ICE