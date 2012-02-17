/*
 * author: Marko Mahnic
 * created: 2010-07-11
 *
 * (c) Copyright 2010 Marko Mahnic. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
package si.unilj.fri.cogx.v11n.test;

import cast.architecture.ManagedComponent;
import si.unilj.fri.cogx.v11n.core.DisplayClient;

import Visualization.TFormFieldMapHolder;

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Map;
import java.util.HashMap;
import java.util.Random;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileNotFoundException;
import java.io.IOException;

public class V11nJavaTestComponent extends ManagedComponent
{
   private class MyDisplayClient extends DisplayClient
   {
      V11nJavaTestComponent m_test = null;
      public MyDisplayClient(V11nJavaTestComponent dataOwner)
      {
         m_test = dataOwner;
      }

      @Override
      public void handleForm(String id, String partId, Map<String, String> fields)
      {
         if (m_test == null) return;
         m_test.log("JAVA handleForm " + id + ":" + partId);
         if (id.equals("v11n.java.setHtmlForm") && partId.equals("101")) {
            if (fields.containsKey("textfield")) {
               m_test.m_textField = fields.get("textfield");
               m_test.log("Got textfield: " + m_test.m_textField);
               m_test.appendMessage(m_test.m_textField);
            }
         }
      }

      @Override
      public boolean getFormData(String id, String partId, TFormFieldMapHolder fields)
      {
         if (m_test == null) return false;
         m_test.log("JAVA getFormData " + id + ":" + partId);
         if (id.equals("v11n.java.setHtmlForm") && partId.equals("101")) {
            fields.value = new HashMap<String, String>();
            fields.value.put("textfield", m_test.m_textField);
            return true;
         }
         return false;
      }

      @Override
      public void handleEvent(Visualization.TEvent event)
      {
         if (m_test == null) return;
         if (event.sourceId.equals("cb.test.onoff")) {
            if (event.data.equals("0")) m_test.m_ckTestValue = 0;
            else m_test.m_ckTestValue = 2;
            if (m_test.m_ckTestValue > 0) m_test.appendMessage("Check box " + event.sourceId + " is ON");
            else m_test.appendMessage("Check box " + event.sourceId + " is OFF");
         }
         if (event.sourceId.equals("button.test")) {
            m_test.appendMessage("Button " + event.sourceId + " PRESSED");
         }
      }

      @Override
      public String getControlState(String ctrlId)
      {
         if (m_test == null) return "";
         if (ctrlId.equals("cb.test.onoff")) {
            if (m_test.m_ckTestValue != 0) return "2";
            else return "0";
         }
         return "";
      }
   }
   
   DisplayClient m_display = new MyDisplayClient(this);
   String m_textField = "A message from Java"; // edited in HTML form
   int m_ckTestValue = 2; // value of a checkbox

   @Override
   protected void configure(java.util.Map<String, String> config)
   {
      m_display.configureDisplayClient(config);
   }

   @Override
   protected void start()
   {
      m_display.connectIceClient(this);
      m_display.installEventReceiver();
   }

   @Override
   protected void runComponent()
   {
      sleepComponent(1000);

      if (true) {
         // A multi-part HTML document.
         // Parts will be added every time the form (setHtmlForm below) is submitted (see handleForm).
         m_display.setHtml("v11n.java.setHtml", "001", "This is a message from V11nJavaTestComponent.");

         // Test of gui elements
         // Messages will be added to the document when events happen (see handleEvent).
         m_display.addCheckBox("v11n.java.setHtml", "cb.test.onoff", "Test On Off");
         m_display.addButton("v11n.java.setHtml", "button.test", "Test Button");
      }

      if (true) {
         // A simple form.
         // Events will be handled in MyDisplayClient.handleForm().
         // Form data will be retreived in MyDisplayClient.getFormData().
         m_display.setHtmlForm("v11n.java.setHtmlForm", "101",
               "Edit me: <input type='text' name='textfield' value='Empty' />");
      }

      if (true) makePusher();
      if (true) makeSvgGraph();

      while (this.isRunning()) {
         sleepComponent(100);
         movePusher();
         updateSvgGraph();
      }
   }

   private int m_msgid = 9000;
   // Called from handleForm() (after a form is submitted)
   // Appends a text message (a html chunk) to an HTML object.
   // Chunks are sorted by their string ID (but the order depends on C++ std::map implementation).
   protected void appendMessage(String message)
   {
      m_msgid += 1;
      m_display.setHtml("v11n.java.setHtml", String.format("%04d", m_msgid), "<br>" + message);
   }

   private void makePusher()
   {
      // A LuaGl script.
      // Load the script from file and send it to the server.
      // Script animation is done in movePusher() called form the main loop below.
      String script = fileAsString("subarchitectures/visualization.sa/src/c++/core/object/gllua/test/pusher.luagl");
      m_display.setLuaGlObject("v11n.java.Pusher", "Pusher", script);
   }

   private int m_moveCount = 0;
   private int m_moverBoxRot = 0;
   Random m_randGen = new Random( 19580427 );
   private void movePusher()
   {
      StringBuffer str = new StringBuffer();
      m_moveCount++;
      if (m_moveCount > 100) m_moveCount = 0;
      if (m_moveCount % 2 == 0) {
         int dir = m_randGen.nextInt() % 4;
         switch (dir) {
            case 0: str.append("move(1,0);\n"); break;
            case 1: str.append("move(0,1);\n"); break;
            case 2: str.append("move(-1,0);\n"); break;
            case 3: str.append("move(0,-1);\n"); break;
         }
      }
      m_moverBoxRot = (m_moverBoxRot + 1) % 36;
      str
         .append(String.format("boxTurn=%d;\n", (m_moverBoxRot * 10)))
         .append("DispList:setDirty('pusher.box.rotation');\n");

      // This will not replace the old script. The new chunk will be evaluated
      // in the appropriate Lua context. The old functions and variables will
      // be kept if they are not redefined by the new script.
      m_display.setLuaGlObject("v11n.java.Pusher", "Pusher", str.toString());
   }

   // The graph is created with 3 chunks: graph border, graph labels and graph
   // data. They are displayed in the order: border - data - labels.
   // (The order is defined with partId, eg. 500_lines; ATM this depends on the
   // implementation of C++ std::map)
   // Border and labels don't change with time so they are prepared at the
   // beginning.
   // The graph data is updated in updateSvgGraph().
   // Every chunk must be a valid SVG document, otherwise the server may crash.
   private int m_GraphData[] = new int[32];
   private void makeSvgGraph()
   {
      StringBuffer str = new StringBuffer();
      str.append("<svg viewbox='0 0 242 162'>");
      str.append("<rect x='0' y='0' width='242' height='162' fill='white' stroke='blue' stroke-width='1' />");
      str.append("<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,120 241,120' />");
      str.append("<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,80 241,80' />");
      str.append("<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,40 241,40' />");
      str.append("</svg>");
      m_display.setObject("v11.java.Graph", "000_background", str.toString());

      str = new StringBuffer();
      str.append("<svg viewbox='0 0 242 162'>");
      str.append(String.format("<text x='2' y='160' font-size='12' fill='blue'>%d</text>", 10));
      str.append(String.format("<text x='2' y='16' font-size='12' fill='blue'>%d</text>", 88));
      str.append("</svg>");
      m_display.setObject("v11.java.Graph", "999_labels", str.toString());
      for(int i = 0; i < 32; i++) m_GraphData[i] = m_randGen.nextInt(70) + 10;
   }

   private int m_GraphPos = 0;
   private void updateSvgGraph()
   {
      m_GraphPos = (m_GraphPos + 1) % 32;
      m_GraphData[m_GraphPos] = m_randGen.nextInt(70) + 10;
      StringBuffer str = new StringBuffer();
      StringBuffer text = new StringBuffer();
      str.append("<svg viewbox='0 0 242 162'>");
      str.append("<polyline fill='none' stroke='none' stroke-width='0' points='0,0 242,162' />");
      str.append("<polyline fill='none' stroke='red' stroke-width='1' points='");
      int i = (m_GraphPos + 1) % 32;
      int k = 0;
      while (i != m_GraphPos) {
         double p = 1.0 - (double) (m_GraphData[i] - 10) / (88.0 - 10.0);
         str.append(String.format("%d,%d ", (int) (240.0*k/32+0.5), (int) (160.0*p+0.5)));
         if (m_GraphData[i] % 10 == 0) {
            text.append(String.format("<text x='%d' y='%d' font-size='10' fill='green'>%d</text>",
                     (int) (240.0*k/32+0.5), (int) (160.0*p+0.5), m_GraphData[i]));
         }
         i = (i + 1) % 32;
         k = k + 1;
      }
      str.append("' />\n");
      m_display.setObject("v11.java.Graph", "500_lines",
            str.toString() + text.toString() + "</svg>");
   }


   public String fileAsString(String fname)
   {
      File file = new File(fname);
      StringBuffer contents = new StringBuffer();
      BufferedReader reader = null;

      try {
         reader = new BufferedReader(new FileReader(file));
         String text = null;

         while ((text = reader.readLine()) != null) {
            contents.append(text)
               .append(System.getProperty(
                        "line.separator"));
         }
      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      finally {
         try {
            if (reader != null) {
               reader.close();
            }
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }

      return contents.toString();
   }

}
// vim:sw=3:ts=8:et
