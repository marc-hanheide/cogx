/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Michael Zillich, Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * $Id$
 */

#ifndef INSPECTABLE_COMPONENT_H
#define INSPECTABLE_COMPONENT_H

#include "CASTDatatypeManager.hpp"

#include <cstdarg>
//#include <balt/core/FrameworkProcess.hpp>
#include <balt/core/FrameworkProcess.hpp>
#include <balt/core/Interfaces.hpp>
#include <cast/cdl/guitypes.hh>
#include <cast/core/CASTException.hpp>

#include "DrawBuffer.hpp"


namespace cast {

  class InspectableComponent : 
    public FrameworkProcess,
    public PullReceiver<cdl::guitypes::DrawBatch>, 
    public PushSender<cdl::guitypes::DrawBatch>
  {
  private:
    /**
     * Connector for pushing data to the GUI if we decide that a redraw
     * should happen NOW.
     */
    PushConnectorOut<cdl::guitypes::DrawBatch> *m_flushConnector;

    /**
     * Internal buffer for storing drawing primitives.
     * This is necessary because Java IDL types can only have arrays of FIXED
     * size.
     */
    DrawBuffer m_buffer;

    /**
     * Copy the draw buffer into a draw batch.
     * This is necessary because Java IDL types can only have arrays of FIXED
     * size.
     */
    void copyDrawBufferToBatch(cdl::guitypes::DrawBatch *batch);

  public:
    InspectableComponent(const std::string &_id);
    virtual ~InspectableComponent();

    virtual void setPushConnector(const std::string &_connectionID,
				  PushConnectorOut<cdl::guitypes::DrawBatch> *_out);

    /**
     * We can process the following types of query:
     * "GetConfig" to tell the GUI who we are.
     * "Redraw2D", "Redraw3D" and "RedrawText", whenever we should redraw 
     * ourselves into the GUI.
     */
    virtual void receivePullQuery(const FrameworkQuery &_query,
				  FrameworkLocalData<cdl::guitypes::DrawBatch> *&_pData);

  protected:
    /**
     * This method is called whenever the GUI contents need to be
     * redrawn. Place all Your 2D drawing commands in here.
     * Override in derived classes.
     */
    virtual void redrawGraphics2D() {};

    /**
     * This method is called whenever the GUI contents need to be
     * redrawn. Place all Your 3D drawing commands in here.
     * Override in derived classes.
     */
    virtual void redrawGraphics3D() {};

    /**
     * This method is called whenever the GUI contents need to be
     * redrawn. Place all Your text printing commands in here.
     * Override in derived classes.
     */
    virtual void redrawGraphicsText() {};

    void clearGraphics();

    /**
     * Indicate to the GUI that we wish to have ourselves redrawn NOW.
     * Note that depending on what the GUI has set as currently visible
     * component the redraw might or might not actually happen
     */
    void redrawGraphicsNow();

    void drawRGBImage(int width, int height, const char *rgbBuffer, int flags);
    void drawRGBImage(int width, int height, const std::vector<char> &rgbBuffer,
		      int flags) throw(CASTException);
    void drawText2D(double x, double y, const std::string &text,
		    int red, int green, int blue, int flags);
    void drawPoint2D(double x, double y, int red, int green, int blue, int flags);
    void drawLine2D(double x1, double y1, double x2, double y2,
		    int red, int green, int blue, int flags);
    void drawRect2D(double xmin, double ymin, double xmax,
		    double ymax, int red, int green, int blue, int flags);
    void drawPolygon2D(const std::vector<double> &x, const std::vector<double> &y,
		       int red, int green, int blue, int flags);

    void drawText3D(double x, double y, double z, const std::string &text,
		    int red, int green, int blue, int flags);
    void drawPoint3D(double x, double y, double z, int red, int green, int blue,
		     int flags);
    void drawLine3D(double x1, double y1, double z1,
		    double x2, double y2, double z2,
		    int red, int green, int blue, int flags);
    // axis-parallel box with center (cx,cy,cz) and sizes (sx,sy,sz)
    void drawBox3D(double cx, double cy, double cz,
		   double sx, double sy, double sz,
		   int red, int green, int blue, int flags);
    // 3D coord frame with position and rotation std::vector (direction = axis, length
    // = angle)
    void drawFrame3D(double px, double py, double pz,
		     double rx, double ry, double rz,
		     int red, int green, int blue, int flags);

    void printText(const std::string &text);
    void printText(const char *format, ...);
  };

} //namespace cast

#endif
