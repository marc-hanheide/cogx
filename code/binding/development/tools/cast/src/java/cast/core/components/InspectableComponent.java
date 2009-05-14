/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
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

package cast.core.components;

import balt.core.data.FrameworkQuery;
import balt.core.processes.FrameworkProcess;
import cast.cdl.guitypes.*;
import cast.ui.UIUtils;
import cast.ui.inspectable.DrawBuffer;
import cast.ui.inspectable.GDBPullInterface.GDBPullReceiver;
import cast.ui.inspectable.GDBPushInterface.GDBPushConnectorOut;
import cast.ui.inspectable.GDBPushInterface.GDBPushSender;

/**
 * Abstract base class for all components that the user interface should
 * have access to. Offers various drawing methods for displaying stuff
 * in the user interface. The key method to override is
 * redrawGraphics(), where all the drawing happens. This method is
 * called whenever the user interface wants to "inspect" (visualise) a
 * component.
 * 
 * @author mxz
 */
abstract public class InspectableComponent extends FrameworkProcess
        implements GDBPullReceiver, GDBPushSender {

    /**
     * Init datatypes used for connections.
     */
    static {
        UIUtils.init();
    }

    /**
     * Connector for pushing data to the GUI if we decide that a redraw
     * should happen NOW.
     */
    GDBPushConnectorOut m_flushConnector;

    /**
     * Internal buffer for storing drawing primitives. This is necessary
     * because Java IDL types can only have arrays of FIXED size.
     */
    private DrawBuffer m_buffer;

    private DrawBatch m_emptyBatch;

    public InspectableComponent(String _id) {
        super(_id);
        m_flushConnector = null;
        m_buffer = new DrawBuffer();
    }

    public void setPushConnector(String _connectionID,
                                 GDBPushConnectorOut _out) {
        m_flushConnector = _out;
    }

    /**
     * We can process two types of query: "GetConfig" to tell the GUI
     * what and how we want to display. "Redraw" whenever we should
     * redraw ourselves into the GUI.
     */
    public DrawBatch receivePullQueryGDB(FrameworkQuery _query) {
        DrawBatch batch = new DrawBatch();
        batch.m_compID = getProcessIdentifier().toString();

        // This query will typically be called once by the GUI
        if (_query.getQuery().equals("GetConfig")) {
            copyDrawBufferToBatch(batch);
            // getConfig()
        }
        // this query is called whenever a 2D redraw is necessary
        else if (_query.getQuery().equals("Redraw2D")) {
            clearGraphics();
            redrawGraphics2D();
            copyDrawBufferToBatch(batch);
        }
        // this query is called whenever a 3D redraw is necessary
        else if (_query.getQuery().equals("Redraw3D")) {
            clearGraphics();
            redrawGraphics3D();
            copyDrawBufferToBatch(batch);
        }
        // this query is called whenever a text redraw is necessary
        else if (_query.getQuery().equals("RedrawText")) {
            clearGraphics();
            redrawGraphicsText();
            copyDrawBufferToBatch(batch);
        }

        // System.out.println(batch.m_image.m_width);
        // System.out.println(batch.m_image.m_height);
        // System.out.println(batch.m_image.m_flags);
        // System.out.println(batch.m_image.m_rgbBuffer.length);

        return batch;
    }

    /**
     * Copy the draw buffer into a draw batch. This is necessary because
     * Java IDL types can only have arrays of FIXED size.
     */
    private void copyDrawBufferToBatch(DrawBatch batch) {

        batch.m_image =
                new RGBImage(m_buffer.m_image.m_width,
                    m_buffer.m_image.m_height,
                    new byte[m_buffer.m_image.m_rgbBuffer.length],
                    m_buffer.m_image.m_flags);
        System.arraycopy(m_buffer.m_image.m_rgbBuffer, 0,
            batch.m_image.m_rgbBuffer, 0,
            m_buffer.m_image.m_rgbBuffer.length);

        batch.m_text2Ds = new Text2D[m_buffer.m_text2Ds.size()];
        m_buffer.m_text2Ds.toArray(batch.m_text2Ds);

        batch.m_point2Ds = new Point2D[m_buffer.m_point2Ds.size()];
        m_buffer.m_point2Ds.toArray(batch.m_point2Ds);

        batch.m_line2Ds = new Line2D[m_buffer.m_line2Ds.size()];
        m_buffer.m_line2Ds.toArray(batch.m_line2Ds);

        batch.m_rect2Ds = new Rect2D[m_buffer.m_rect2Ds.size()];
        m_buffer.m_rect2Ds.toArray(batch.m_rect2Ds);

        batch.m_poly2Ds = new Polygon2D[m_buffer.m_poly2Ds.size()];
        m_buffer.m_poly2Ds.toArray(batch.m_poly2Ds);

        batch.m_text3Ds = new Text3D[m_buffer.m_text3Ds.size()];
        m_buffer.m_text3Ds.toArray(batch.m_text3Ds);

        batch.m_point3Ds = new Point3D[m_buffer.m_point3Ds.size()];
        m_buffer.m_point3Ds.toArray(batch.m_point3Ds);

        batch.m_line3Ds = new Line3D[m_buffer.m_line3Ds.size()];
        m_buffer.m_line3Ds.toArray(batch.m_line3Ds);

        batch.m_box3Ds = new Box3D[m_buffer.m_box3Ds.size()];
        m_buffer.m_box3Ds.toArray(batch.m_box3Ds);

        batch.m_frame3Ds = new Frame3D[m_buffer.m_frame3Ds.size()];
        m_buffer.m_frame3Ds.toArray(batch.m_frame3Ds);

        batch.m_texts = new String[m_buffer.m_texts.size()];
        m_buffer.m_texts.toArray(batch.m_texts);
    }

    /**
     * This method is called (typically once by the GUI at startup) to
     * obtain information on what and how the component wishes to
     * display in the GUI. Override in derived classes.
     */
    // abstract protected void getConfig();
    /**
     * Clears all buffered drawing primitives.
     */
    private void clearGraphics() {
        m_buffer.clear();
    }

    /**
     * This method is called whenever the GUI contents need to be
     * redrawn. Place all Your 2D drawing commands in here. Override in
     * derived classes.
     */
    protected void redrawGraphics2D() {}

    /**
     * This method is called whenever the GUI contents need to be
     * redrawn. Place all Your 3D drawing commands in here. Override in
     * derived classes.
     */
    protected void redrawGraphics3D() {}

    /**
     * This method is called whenever the GUI contents need to be
     * redrawn. Place all Your text printing commands in here. Override
     * in derived classes.
     */
    protected void redrawGraphicsText() {}

    /**
     * Indicate to the GUI that we wish to have ourselves redrawn NOW.
     * Note that depending on what the GUI has set as currently visible
     * component the redraw might or might not actually happen
     */
    public void redrawGraphicsNow() {
        if (m_flushConnector != null) {
            
            //lazy creation of empty batch
            if (m_emptyBatch == null) {
                m_emptyBatch =
                        new DrawBatch("", new RGBImage(0, 0,
                            new byte[0],
                            NONE.value), new Text2D[0], new Point2D[0],
                            new Line2D[0], new Rect2D[0],
                            new Polygon2D[0], new Text3D[0],
                            new Point3D[0], new Line3D[0],
                            new Box3D[0], new Frame3D[0], new String[0]);
            }
            // Just send an empty batch to "wake up" GUI. The GUI will
            // then
            // use its
            // normal pull connection to do the actual redraw.
            // Downside: two communiction steps
            // Upside: If the GUI is not currently displaying this
            // component
            // (so that
            // nothing should actually be drawn) we don't waste
            // bandwidth
            // sending lots of
            // drawing primitives into nirvana.
            m_flushConnector.push(getProcessIdentifier().toString(),
                m_emptyBatch);
        }
    }

    public void drawRGBImage(int width,
                             int height,
                             byte[] rgbBuffer,
                             int flags) {
        m_buffer.m_image =
                new RGBImage(width, height, rgbBuffer, flags);
    }

    public void drawText2D(double x,
                           double y,
                           String text,
                           int red,
                           int green,
                           int blue,
                           int flags) {
        m_buffer.m_text2Ds.add(new Text2D(x, y, text, new RGBColor(red,
            green, blue), flags));
    }

    public void drawPoint2D(double x,
                            double y,
                            int red,
                            int green,
                            int blue,
                            int flags) {
        m_buffer.m_point2Ds.add(new Point2D(x, y, new RGBColor(red,
            green, blue), flags));
    }

    public void drawLine2D(double x1,
                           double y1,
                           double x2,
                           double y2,
                           int red,
                           int green,
                           int blue,
                           int flags) {
        m_buffer.m_line2Ds.add(new Line2D(x1, y1, x2, y2, new RGBColor(
            red, green, blue), flags));
    }

    public void drawRect2D(double xmin,
                           double ymin,
                           double xmax,
                           double ymax,
                           int red,
                           int green,
                           int blue,
                           int flags) {
        m_buffer.m_rect2Ds.add(new Rect2D(xmin, ymin, xmax, ymax,
            new RGBColor(red, green, blue), flags));
    }

    public void drawPolygon2D(double[] x,
                              double[] y,
                              int red,
                              int green,
                              int blue,
                              int flags) {
        m_buffer.m_poly2Ds.add(new Polygon2D(x, y, new RGBColor(red,
            green, blue), flags));
    }

    public void drawText2D(double x,
                           double y,
                           double z,
                           String text,
                           int red,
                           int green,
                           int blue,
                           int flags) {
        m_buffer.m_text3Ds.add(new Text3D(x, y, z, text, new RGBColor(
            red, green, blue), flags));
    }

    public void drawPoint3D(double x,
                            double y,
                            double z,
                            int red,
                            int green,
                            int blue,
                            int flags) {
        m_buffer.m_point3Ds.add(new Point3D(x, y, z, new RGBColor(red,
            green, blue), flags));
    }

    public void drawLine3D(double x1,
                           double y1,
                           double z1,
                           double x2,
                           double y2,
                           double z2,
                           int red,
                           int green,
                           int blue,
                           int flags) {
        m_buffer.m_line3Ds.add(new Line3D(x1, y1, z1, x2, y2, z2,
            new RGBColor(red, green, blue), flags));
    }

    public void drawFrame3D(double px,
                            double py,
                            double pz,
                            double rx,
                            double ry,
                            double rz,
                            int red,
                            int green,
                            int blue,
                            int flags) {
        m_buffer.m_frame3Ds.add(new Frame3D(px, py, pz, rx, ry, rz,
            new RGBColor(red, green, blue), flags));
    }

    public void drawBox3D(double cx,
                          double cy,
                          double cz,
                          double sx,
                          double sy,
                          double sz,
                          int red,
                          int green,
                          int blue,
                          int flags) {
        m_buffer.m_box3Ds.add(new Box3D(cx, cy, cz, sx, sy, sz,
            new RGBColor(red, green, blue), flags));
    }

    public void printText(String str) {
        m_buffer.m_texts.add(str);
    }
}
