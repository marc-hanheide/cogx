#ifndef _DISPLAY_SERVER_ICE_
#define _DISPLAY_SERVER_ICE_

#include <Ice/Identity.ice>
#include <cast/slice/CDL.ice>

module Visualization
{
   const int V11NSTANDALONEPORT = 10511;
   const string V11NSTANDALONENAME = "StandaloneDisplayServer";

   sequence<byte> ByteSeq;
   sequence<double> FloatSeq;
   sequence<string> StringSeq;

   struct Quaternion {
      double x;
      double y;
      double z;
      double w;
   };

   struct ActionInfo {
      string id;
      string label;

      // iconLabel Optional, defaults to label
      string iconLabel;

      string tooltip;

      // iconSvg Optional
      string iconSvg;

      // iconImage Optional, iconSvg takes precedence
      ByteSeq iconImage;

      bool checkable;
   };

   dictionary<string, string> TFormFieldMap;

   enum ViewType { VtGraphics, VtOpenGl, VtHtml };

   interface DisplayInterface
   {
      // Used internally
      void resetServer(int secret);

      // Get the parameters to connect to a remote Display Server
      void getStandaloneHost(out string hostname);

      // Create views to show objects
      void createView(string viewId, ViewType type, StringSeq objects);
      void enableDefaultView(string objectId, bool enable);

      // Create objects in the display server
      void setObject(string id, string partId, string svgObject);
      void setObjectTransform2D(string id, string partId, FloatSeq matrix33);

      // 3 channels for RGB, 1 channel for GS
      void setRawImage(string id, int width, int height, int channels, ByteSeq data);

      // Formats supported by Qt
      void setCompressedImage(string id, ByteSeq data, string format);

      // Set a serialized tgRenderModel
      void setTomGineObject(string id, string partId, ByteSeq data);
      void setObjectPose3D(string id, string partId, double x, double y, double z, Quaternion rotation);

      // Set a LuaGL script
      void setLuaGlObject(string id, string partId, string script);

      // Set an HTML chunk
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

      // Remove objects and object parts
      void removeObject(string id);
      void removePart(string id, string partId);

      // Event handlers need to subscribe
      // TODO: parameter: which views to watch
      // nah: added extra parameters to
      void addClient(Ice::Identity ident, string host, int port);

      // TODO: a checkbox has an initial value (0, 1, 2)
      // TODO: ActionInfo parameter
      void addCheckBox(Ice::Identity ident, string viewId, string ctrlId, string label);

      // TODO: ActionInfo parameter
      void addButton(Ice::Identity ident, string viewId, string ctrlId, string label);

      void addAction(Ice::Identity ident, string viewId, ActionInfo info);
      void enableMouseEvents(Ice::Identity ident, string viewId, bool enabled);

      // Create a Qt dialog. Only one dialog with a unique dialogId can exist. Multiple
      // components may try to add a dialog with the same dialogId, but only the first
      // one will succeed.
      //
      // @param designCode - Qt4 designer code (xml)
      // @param scriptCode - JavaScript code with the dialog logic
      // @param constructorName - the name of a JS 'constructor' function and optionally
      //    the name of the JS global object that will be created.
      //    case 1: 'Calculator'
      //       This will create an instance of the Calculator class named '_Calculator_'.
      //    case 2: 'Calculator calc'
      //       The Calculator instance will be accessible through a global name 'calc'.
      void addDialog(Ice::Identity ident, string dialogId, string designCode,
            string scriptCode, string constructorName);
      void execInDialog(string dialogId, string scriptCode);
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

      // TODO: the value should be JSON; now it's just a string representation (QtVariant.toString())
      void onDialogValueChanged(string dialogId, string name, string value);
      void handleDialogCommand(string dialogId, string command, string params);
   };
};

#endif
// vim:sw=3:ts=8:et:ai
