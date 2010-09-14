#ifndef _DISPLAY_SERVER_ICE_
#define _DISPLAY_SERVER_ICE_

#include <Ice/Identity.ice>
#include <cast/slice/CDL.ice>
#include <Video.ice>
#include <Math.ice>

module Visualization
{

   const int V11NSTANDALONEPORT = 10511;
   const string V11NSTANDALONENAME = "StandaloneDisplayServer";

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

      // iconLabel Optional, defaults to label
      string iconLabel;

      // iconSvg Optional
      string iconSvg;

      // iconImage Optional, iconSvg takes precedence
      ByteSeq iconImage;

      bool checkable;
   };

   dictionary<string, string> TFormFieldMap;

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

      // A HTML chunk that can send onClick evnets to the owner.
      // Add @@ONCLICK@@('unique.id') to clickable HTML elements. The tag will be replaced
      // by the DisplayServer.
      void setActiveHtml(Ice::Identity ident, string id, string partId, string htmlData);

      // Create an active HTML form that sends the submitted data to the client.
      // Note: don't use the <form> tag in htmlData, it will be added.
      // Note: @@ONCLICK@@ events are also supported.
      void setHtmlForm(Ice::Identity ident, string id, string partId, string htmlData);

      // Set html form data. This function usually doesn't need to be called since
      // the form data is retreived with EventReceiver::getFormData() when necessary.
      // If the application needs to reload the data while the form is being edited
      // (eg. after a CogxJsSendValue event) this method can be used.
      void setHtmlFormData(string id, string partId, TFormFieldMap fields);

      // Event handlers need to subscribe
      // TODO: parameter: which views to watch
      // nah: added extra parameters to
      void addClient(Ice::Identity ident, string host, int port);

      // TODO: a checkbox has an initial value (0, 1, 2)
      // TODO: ActionInfo parameter
      void addCheckBox(Ice::Identity ident, string viewId, string ctrlId, string label);

      // TODO: ActionInfo parameter
      void addButton(Ice::Identity ident, string viewId, string ctrlId, string label);

      void addToolButton(Ice::Identity ident, string viewId, string ctrlId, ActionInfo info);
      void enableMouseEvents(Ice::Identity ident, string viewId, bool enabled);
   };

   enum EEventType
   {
      // evButtonClick - A normal button was clicked
      evButtonClick,

      // evCheckBoxChange - data = state
      evCheckBoxChange,

      // evDropListChange - data = list item text
      evDropListChange,

      // evMouseClick - source = view, data = button, (x,y) = position
      evMouseClick,

      // evHtmlOnClick - An HTML element was clicked
      evHtmlOnClick
   };

   struct TEvent
   {
      EEventType type;

      // objectId - viewId from addCheckBox etc. / id from setActiveHtml etc.
      string objectId;

      // partId
      string partId;

      // sourceId - ctrlId from addCheckBox etc. / id from @@ONCLICK@@
      string sourceId;

      string data;

      // Position of the mouse click inside the control
      float x;
      float y;
   };

   interface EventReceiver
   {
      void handleEvent(TEvent event);
      void handleForm(string id, string partId, TFormFieldMap fields);

      // getControlState XXX: maybe add param what; maybe add id, partId
      string getControlState(string ctrlId);

      // Get data to fill the form when first displayed; returns false if not supported
      bool getFormData(string id, string partId, out TFormFieldMap fields);
   };
};

#endif
// vim:sw=3:ts=8:et:ai
