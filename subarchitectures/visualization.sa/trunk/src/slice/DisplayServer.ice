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

   struct ActionInfo {
      string label;
      string iconLabel;   // Optional, defaults to label
      string iconSvg;     // Optional
      ByteSeq iconImage;  // Optional, iconSvg takes precedence
      bool checkable;
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

      // Pass a Lua script
      void setLuaGlObject(string id, string partId, string script);

      // Pass an HTML chunk
      void setHtml(string id, string partId, string htmlData);
      void setHtmlHead(string id, string partId, string htmlData);

      // Create an active HTML form that sends the data to the client.
      // Note: don't use the <form> tag in htmlData, it will be added.
      void setHtmlForm(Ice::Identity ident, string id, string partId, string htmlData);

      // Event handlers need to subscribe
      // TODO: parameter: which views to watch
      void addClient(Ice::Identity ident);

      // TODO: a checkbox has an initial value (0, 1, 2)
      void addCheckBox(Ice::Identity ident, string viewId, string ctrlId, string label); // TODO: ActionInfo
      void addButton(Ice::Identity ident, string viewId, string ctrlId, string label); // TODO: ActionInfo
      void addToolButton(Ice::Identity ident, string viewId, string ctrlId, ActionInfo info);
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
   
   dictionary<string, string> TFormFieldMap;
   interface EventReceiver
   {
      void handleEvent(TEvent event);
      string getControlState(string ctrlId); // XXX: maybe add param what
      void handleForm(string formId, TFormFieldMap fields);
      void getFormData(string formId, out TFormFieldMap fields);
   };
};

#endif
// vim:sw=3:ts=8:et:ai
