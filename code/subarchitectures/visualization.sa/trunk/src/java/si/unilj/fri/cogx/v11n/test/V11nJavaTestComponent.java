/*
 * author: Marko Mahnič
 * created: 2010-07-11
 *
 * © Copyright 2010 Marko Mahnič. 
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

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Map;

public class V11nJavaTestComponent extends ManagedComponent
{
   private class MyDisplayClient extends DisplayClient
   {
      @Override
      public void handleForm(String id, String partId, Map<String, String> fields)
      {
      }
   }
   
   DisplayClient m_display = new MyDisplayClient();

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
         m_display.setHtml("v11.java.setHtml", "001", "This is a message from V11nJavaTestComponent.");
      }

      if (true) {
         m_display.setHtmlForm("v11.java.setHtmlForm", "101",
           "Edit me: <input type='text' value='Text from java' />");
      }

      while (this.isRunning()) {
         sleepComponent(200);
      }
   }
}
// vim:sw=3:ts=8:et
