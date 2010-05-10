#ifndef _DISPLAY_SERVER_ICE_
#define _DISPLAY_SERVER_ICE_

#include <Ice/Identity.ice>
#include <cast/slice/CDL.ice>
#include <Video.ice>
#include <Math.ice>

module Visualization
{
   interface DisplayInterface
   {
      void setObject(string id, string partId, string objectXml);
      void setObjectTransform(string id, string partId, cogx::Math::Matrix33 transform);
      void setImage(string id, Video::Image image);

      // 3 channels for RGB, 1 channel for GS
      void setRawImage(string id, int width, int height, int channels, Video::ByteSeq data);

      void setCompressedImage(string id, Video::ByteSeq data, string format);

      // Event handlers need to subscribe
      // TODO: parameter: which views to watch
      void addClient(Ice::Identity ident);

      // TODO: a checkbox has an initial value (0, 1, 2)
      void addCheckBox(Ice::Identity ident, string viewId, string ctrlId, string label);
      void addButton(Ice::Identity ident, string viewId, string ctrlId, string label);
   };

   enum EEventType
   {
      evButtonClick, evCheckBoxChange, evDropListChange
   };

   struct TEvent
   {
      EEventType type;
      string sourceId;
      string data;
   };

   interface EventReceiver
   {
      void handleEvent(TEvent event);
      // TODO: getControlState(ctrl-id-list) returns dictionary[ctrl-id, value]
   };
};

#endif
