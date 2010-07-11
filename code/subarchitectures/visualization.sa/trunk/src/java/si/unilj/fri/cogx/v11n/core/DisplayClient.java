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
package si.unilj.fri.cogx.v11n.core;

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------
import cast.architecture.*;
import cast.cdl.*;
import cast.core.*;
import cast.CASTException;

//-----------------------------------------------------------------
// VISUALIZATION IMPORTS
//-----------------------------------------------------------------
//import si.unilj.fri.cogx.Visualization.DisplayInterface;
//import si.unilj.fri.cogx.Visualization.DisplayInterfacePrx;
//import si.unilj.fri.cogx.Visualization.*;
import Visualization.*; // ICE interfaces

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.lang.StringBuilder;
import java.util.Map;
import java.util.Iterator;

public class DisplayClient
{
   private String m_ServerName = "display.srv";
   private DisplayInterfacePrx m_Server = null;

   public void configureDisplayClient(java.util.Map<String, String> config)
   {
      if (config.containsKey("--displayserver")) {
         m_ServerName = config.get("--displayserver");
      }
   }

   public void connectIceClient(CASTComponent owner)
   {
      // a)
      try {
         // this only works if you don't play with [["java:package:..."]] in ice files
         m_Server = owner.getIceServer(m_ServerName, DisplayInterface.class, DisplayInterfacePrx.class);
      }
      catch (CASTException e) {
         e.printStackTrace();
      }
   }

   public void setImage(String id, int width, int height, int channels, byte data[])
   {
      if (m_Server == null) return;
      m_Server.setRawImage(id, width, height, channels, data);
   }
   //void setImage(String id, Video.Image image); 

   public void setObject(String id, String partId, String svgObject)
   {
      if (m_Server == null) return;
      m_Server.setObject(id, partId, svgObject);
   }

   public void setHtml(String id, String partId, String htmlData)
   {
      if (m_Server == null) return;
      m_Server.setHtml(id, partId, htmlData);
   }

   public void setHtmlHead(String id, String partId, String htmlData)
   {
      if (m_Server == null) return;
      m_Server.setHtmlHead(id, partId, htmlData);
   }

   //public void setHtmlForm(String id, String partId, String htmlData)
   //{
   //   if (m_Server == null) return;
   //   m_Server.setHtmlForm(ident, id, partId, htmlData);
   //}
}
// vim:sw=3:ts=8:et
