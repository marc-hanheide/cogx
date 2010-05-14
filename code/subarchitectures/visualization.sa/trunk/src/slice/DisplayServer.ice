#ifndef _DISPLAY_SERVER_ICE_
#define _DISPLAY_SERVER_ICE_

#include <Ice/Identity.ice>
#include <cast/slice/CDL.ice>
#include <Video.ice>
#include <Math.ice>

module Visualization
{
   sequence<byte> ByteSeq;
   sequence<double> FloatSeq;

   struct Quaternion {
      double x;
      double y;
      double z;
      double w;
   };

   interface DisplayInterface
   {
      void setObject(string id, string partId, string svgObject);
      void setObjectTransform2D(string id, string partId, FloatSeq matrix33);

      void setImage(string id, Video::Image image);

      // 3 channels for RGB, 1 channel for GS
      void setRawImage(string id, int width, int height, int channels, ByteSeq data);

      // Formats supported by Qt
      void setCompressedImage(string id, ByteSeq data, string format);

      // Pass a serialized tgRenderModel
      void setTomGineObject(string id, string partId, ByteSeq data);
      void setObjectPose3D(string id, string partId, cogx::Math::Vector3 position, Quaternion rotation);

      // Event handlers need to subscribe
      // TODO: parameter: which views to watch
      void addClient(Ice::Identity ident);

      // TODO: a checkbox has an initial value (0, 1, 2)
      void addCheckBox(Ice::Identity ident, string viewId, string ctrlId, string label);
      void addButton(Ice::Identity ident, string viewId, string ctrlId, string label);
      void addToolButton(Ice::Identity ident, string viewId, string ctrlId, string label, string svgIcon);
      void enableMouseEvents(Ice::Identity ident, string viewId, bool enabled);
   };

   enum EEventType
   {
      evButtonClick,
      evCheckBoxChange, // data = state
      evDropListChange, // data = list item text
      evMouseClick      // source = view, data = button, (x,y) = position
   };

   struct TEvent
   {
      EEventType type;
      string sourceId;
      string data;
      float x;
      float y; // Position of the mouse click inside the control
   };

   interface EventReceiver
   {
      void handleEvent(TEvent event);
      // TODO: getControlState(ctrl-id-list) returns dictionary[ctrl-id, value]
   };
};

#endif
