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

/**
 * 
 */
package cast.ui;

import java.io.*;
import java.nio.ByteBuffer;
import java.util.*;

import javax.media.opengl.*;
import javax.media.opengl.glu.GLU;

import org.eclipse.swt.SWT;
import org.eclipse.swt.SWTException;
import org.eclipse.swt.custom.*;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.layout.*;
import org.eclipse.swt.opengl.GLCanvas;
import org.eclipse.swt.opengl.GLData;
import org.eclipse.swt.widgets.*;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.data.FrameworkQuery;
import cast.cdl.guitypes.*;
import cast.cdl.ui.*;
import cast.configuration.ArchitectureConfiguration;
import cast.ui.architecture.*;
import cast.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnectorOut;
import cast.ui.architecture.views.ArchitectureView;
import cast.ui.inspectable.GDBPullInterface.GDBPullConnectorOut;
import cast.ui.inspectable.GDBPullInterface.GDBPullSender;
import cast.ui.inspectable.GDBPushInterface.GDBPushReceiver;

import com.sun.opengl.util.GLUT;

/**
 * @author nah
 */
public class UnifiedGUI extends UIServer implements GDBPullSender,
		GDBPushReceiver {

	private final class ScheduleUpdateRunnable implements Runnable {
		private final int rate;

		private ScheduleUpdateRunnable(int rate) {
			this.rate = rate;
		}

		public void run() {
			m_display.timerExec(rate, new TimedUpdateRunnable(rate));
		}
	}

	private final class TimedUpdateRunnable implements Runnable {
		private final int rate;

		private TimedUpdateRunnable(int rate) {
			this.rate = rate;
		}

		public void run() {
//			System.out.println(".run()");
			try {
				redrawAllViews();
			} catch (SWTException e) {
				// e.printStackTrace();
				System.out.println("Exception in GUI runnable 1");
				e.printStackTrace();
			}

			m_display.asyncExec(new ScheduleUpdateRunnable(rate));
		}
	}

	static class AppendRunnable implements Runnable {

		private String m_text;

		private Text m_output;

		public AppendRunnable(String _text, Text _output) {
			m_text = _text;
			m_output = _output;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.lang.Runnable#run()
		 */
		public void run() {

			// System.out.println("appending: " + m_text);
			m_output.append(m_text);
		}

	}

	private class GUIOutputStream extends OutputStream {

		private StringBuffer m_buffer;

		/**
		 * 
		 */
		public GUIOutputStream() {
			m_buffer = new StringBuffer();
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.io.OutputStream#flush()
		 */
		@Override
		public void flush() throws IOException {
			append(m_buffer.toString());
			m_buffer = new StringBuffer();
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.io.OutputStream#write(int)
		 */
		@Override
		public void write(int _b) throws IOException {
			// System.out.println("write: " + _b);

			m_buffer.append((char) _b);
			if ((char) _b == '\n') {
				flush();
			}
		}
	}

	private class MonitorUpdaterThread extends Thread {

		private boolean m_bRun = true;

		/**
		 * 
		 */
		private void stopUpdating() {
			m_bRun = false;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see java.lang.Thread#run()
		 */
		@Override
		public void run() {
			try {
				while (m_bRun) {
					synchronized (this) {
						wait(100);
					}

					// TODO Configure to change based on visibility

					for (ComponentMonitor monitor : m_monitors.values()) {
						// System.out.println("updating: " +
						// monitor.getID());
						monitor.update();
					}
					if (m_architectureView != null) {
						m_architectureView.updateView();
					}
				}
			} catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			} catch (CASTUIException e) {
				e.printStackTrace();
				System.exit(1);
			}

		}
	}

	private class View2D extends GLCanvas {

		private final GLContext m_context;

		private final GLU m_glu = new GLU();

		private final GLUT m_glut = new GLUT();

		public View2D(Composite parent, GLData glData) {
			super(parent, SWT.NO_BACKGROUND, glData);
			setCurrent();
			m_context = GLDrawableFactory.getFactory()
					.createExternalGLContext();
			m_context.makeCurrent();

			GL gl = m_context.getGL();

			gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			gl.glDisable(GL.GL_DEPTH_TEST);
			gl.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1);
			gl.glPixelZoom(1.0f, -1.0f);

			m_context.release();

			addListener(SWT.Resize, new Listener() {

				public void handleEvent(Event event) {
					resizeGL();
				}
			});

			addListener(SWT.Paint, new Listener() {

				public void handleEvent(Event event) {
					redrawGL();
				}
			});
		}

		private void resizeGL() {
			setCurrent();
			m_context.makeCurrent();
			Rectangle bounds = getBounds();
			GL gl = m_context.getGL();

			gl.glViewport(0, 0, bounds.width, bounds.height);
			gl.glMatrixMode(GL.GL_PROJECTION);
			gl.glLoadIdentity();
			gl.glScaled(1.0, -1.0, 1.0); // y points down
			m_glu.gluOrtho2D(0, bounds.width, 0, bounds.height);
			gl.glMatrixMode(GL.GL_MODELVIEW);
			gl.glLoadIdentity();

			m_context.release();
		}

		/**
		 * Prepare (set context, set view transformation) and clear.
		 */
		private void startRedrawGL() {
			setCurrent();
			m_context.makeCurrent();
			GL gl = m_context.getGL();

			gl.glClear(GL.GL_COLOR_BUFFER_BIT);
			gl.glMatrixMode(GL.GL_MODELVIEW);
			gl.glLoadIdentity();
		}

		/**
		 * Once finished with drawing, swap buffers to make changes visible.
		 */
		private void finishRedrawGL() {
			swapBuffers();
			m_context.release();
		}

		/**
		 * Redraw the 2D view of the currently selected component.
		 */
		private void redrawGL() {
			String comp = getSelectedComponent();
			if (comp != null) {
				startRedrawGL();
				try {
					GDBPullConnectorOut conn = m_componentConnectors.get(comp);
					DrawBatch batch = conn.pull(new FrameworkQuery(
							getProcessIdentifier(), "Redraw2D"));
					if (batch != null)
						drawBatch2D(batch);
				} catch (FrameworkConnectionException e) {
					System.out.println("Connection to component failed: " + e);
				}
				finishRedrawGL();
			}
		}

		/**
		 * Draw the 2D contents of a draw batch into the 2D view.
		 */
		private void drawBatch2D(DrawBatch batch) {
			GL gl = m_context.getGL();

			// the image is drawn first, so it acts a background, all
			// other
			// drawing primitives being overlaid
			if (batch.m_image != null && batch.m_image.m_rgbBuffer.length != 0)
				drawRGBImage(gl, batch.m_image);

			for (int i = 0; i < batch.m_text2Ds.length; i++)
				drawText2D(gl, batch.m_text2Ds[i]);

			for (int i = 0; i < batch.m_point2Ds.length; i++)
				drawPoint2D(gl, batch.m_point2Ds[i]);

			for (int i = 0; i < batch.m_line2Ds.length; i++)
				drawLine2D(gl, batch.m_line2Ds[i]);

			for (int i = 0; i < batch.m_rect2Ds.length; i++)
				drawRect2D(gl, batch.m_rect2Ds[i]);

			for (int i = 0; i < batch.m_poly2Ds.length; i++)
				drawPolygon2D(gl, batch.m_poly2Ds[i]);
		}

		private void drawText2D(GL gl, Text2D text) {
			gl.glColor3d(text.m_color.m_r / 255., text.m_color.m_g / 255.,
					text.m_color.m_b / 255.);
			gl.glRasterPos2d(text.m_x, text.m_y);
			m_glut.glutBitmapString(GLUT.BITMAP_HELVETICA_10, text.m_text);
		}

		private void drawPoint2D(GL gl, Point2D point) {
			gl.glColor3d(point.m_color.m_r / 255., point.m_color.m_g / 255.,
					point.m_color.m_b / 255.);
			if ((point.m_flags & FAT.value) != 0)
				gl.glPointSize(3.f);
			else
				gl.glPointSize(1.f);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex2d(point.m_x, point.m_y);
			gl.glEnd();
		}

		private void drawLine2D(GL gl, Line2D line) {
			gl.glColor3d(line.m_color.m_r / 255., line.m_color.m_g / 255.,
					line.m_color.m_b / 255.);
			if ((line.m_flags & FAT.value) != 0)
				gl.glLineWidth(3.f);
			else
				gl.glLineWidth(1.f);
			gl.glBegin(GL.GL_LINES);
			gl.glVertex2d(line.m_x1, line.m_y1);
			gl.glVertex2d(line.m_x2, line.m_y2);
			gl.glEnd();
		}

		private void drawRect2D(GL gl, Rect2D rect) {
			gl.glColor3d(rect.m_color.m_r / 255., rect.m_color.m_g / 255.,
					rect.m_color.m_b / 255.);
			if ((rect.m_flags & FAT.value) != 0)
				gl.glLineWidth(3.f);
			else
				gl.glLineWidth(1.f);
			if ((rect.m_flags & FILLED.value) != 0)
				gl.glBegin(GL.GL_POLYGON);
			else
				gl.glBegin(GL.GL_LINE_LOOP);
			gl.glVertex2d(rect.m_xmin, rect.m_ymin);
			gl.glVertex2d(rect.m_xmax, rect.m_ymin);
			gl.glVertex2d(rect.m_xmax, rect.m_ymax);
			gl.glVertex2d(rect.m_xmin, rect.m_ymax);
			gl.glEnd();
		}

		private void drawPolygon2D(GL gl, Polygon2D poly) {
			gl.glColor3d(poly.m_color.m_r / 255., poly.m_color.m_g / 255.,
					poly.m_color.m_b / 255.);
			if ((poly.m_flags & FAT.value) != 0)
				gl.glLineWidth(3.f);
			else
				gl.glLineWidth(1.f);
			if ((poly.m_flags & FILLED.value) != 0)
				gl.glBegin(GL.GL_POLYGON);
			else
				gl.glBegin(GL.GL_LINE_LOOP);
			for (int i = 0; i < poly.m_x.length; i++)
				gl.glVertex2d(poly.m_x[i], poly.m_y[i]);
			gl.glEnd();
		}

		private void drawRGBImage(GL gl, RGBImage img) {
			ByteBuffer bb = ByteBuffer.allocate(img.m_rgbBuffer.length);
			bb.put(img.m_rgbBuffer);
			bb.position(0);
			bb.limit(bb.capacity());
			gl.glRasterPos2d(0., 0.);
			gl.glDrawPixels(img.m_width, img.m_height, GL.GL_RGB,
					GL.GL_UNSIGNED_BYTE, bb);
		}

	}

	class View3D extends GLCanvas {

		private final GLContext m_context;

		private final GLU m_glu = new GLU();

		private final GLUT m_glut = new GLUT();

		/*
		 * virtual camera position/orientation
		 */
		private double m_cam_trans[] = new double[3];

		private double m_cam_rot[] = new double[3];

		/*
		 * last mouse position for tracking mouse
		 */
		private int m_last_mouse_x;

		private int m_last_mouse_y;

		// size of the ground plane, i.e. the part of it that will be
		// drawn
		private static final double GROUND_SIZE_X = 1.7;

		private static final double GROUND_SIZE_Y = 1.8;

		// we have 5x5 dots on the table spaced dots 15 cm apart (in x and y
		// direction)
		private static final double GROUND_GRID_SIZE = 0.150;

		private static final int GROUND_NUMDOTS_X = 5;

		private static final int GROUND_NUMDOTS_Y = 5;

		// HACK: for arm mounted on B21, the groundplane (= table) is 9 cm
		// below Katana base
		private static final double GROUND_HEIGHT = 0.000;

		public View3D(Composite parent, GLData glData) {
			super(parent, SWT.NO_BACKGROUND, glData);
			setCurrent();
			m_context = GLDrawableFactory.getFactory()
					.createExternalGLContext();
			m_context.makeCurrent();
			GL gl = m_context.getGL();
			gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			gl.glEnable(GL.GL_DEPTH_TEST);
			m_context.release();

			addListener(SWT.Resize, new Listener() {

				public void handleEvent(Event event) {
					resizeGL();
				}
			});

			addListener(SWT.Paint, new Listener() {

				public void handleEvent(Event event) {
					redrawGL();
				}
			});

			addListener(SWT.MouseMove, new Listener() {

				public void handleEvent(Event e) {
					handleMousemove3D(e);
				}
			});

		}

		/**
		 * If the mouse was moved in the 3D view, translate and rotate.
		 */
		private void handleMousemove3D(Event e) {

			double trans_scale = 200., rot_scale = 1.;
			int delta_x = e.x - m_last_mouse_x;
			int delta_y = e.y - m_last_mouse_y;
			boolean do_redraw = false;

			if (e.stateMask == SWT.BUTTON1) {
				m_cam_trans[0] += ((double) delta_x) / trans_scale;
				m_cam_trans[1] -= ((double) delta_y) / trans_scale;
				do_redraw = true;
			} else if (e.stateMask == SWT.BUTTON2) {
				m_cam_trans[2] -= ((double) delta_y) / trans_scale;
				do_redraw = true;
			} else if (e.stateMask == SWT.BUTTON3) {
				m_cam_rot[0] += (double) delta_x / rot_scale;
				m_cam_rot[1] += (double) delta_y / rot_scale;
				do_redraw = true;
			}
			m_last_mouse_x = e.x;
			m_last_mouse_y = e.y;
			if (do_redraw) {
				redrawGL();
				// redraw();
				// update();
				// setRedraw(true);

			}
		}

		private void resizeGL() {
			setCurrent();
			m_context.makeCurrent();
			Rectangle bounds = m_canvas3.getBounds();
			GL gl = m_context.getGL();

			gl.glViewport(0, 0, bounds.width, bounds.height);
			gl.glMatrixMode(GL.GL_PROJECTION);
			gl.glLoadIdentity();
			// viewing frustrum, note all values are in m.
			m_glu
					.gluPerspective(60., bounds.width / bounds.height, 0.001,
							100.);
			gl.glMatrixMode(GL.GL_MODELVIEW);
			gl.glLoadIdentity();

			m_context.release();
		}

		/**
		 * Prepare (set context, set view transformation) and clear.
		 */
		private void startRedrawGL() {
			setCurrent();
			m_context.makeCurrent();
			GL gl = m_context.getGL();

			gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
			gl.glMatrixMode(GL.GL_MODELVIEW);
			gl.glLoadIdentity();
			// TODO: do this viewpoint stuff properly
			gl.glTranslated(m_cam_trans[0], m_cam_trans[1],
					m_cam_trans[2] + 1.0); // HACK: adding 1 to get "nearer"
			m_glu.gluLookAt(-1.0, -2.0, 1.0, 0.0, 0.0, 0.0, 0., 0., 1.0);
			gl.glRotated(m_cam_rot[0], 0., 0., 1.);

			gl.glColor3d(1.0, 1.0, 0.0);
			drawCoordFrame(gl);
			drawGroundPlane(gl);
		}

		/**
		 * Once finished with drawing, swap buffers to make changes visible.
		 */
		private void finishRedrawGL() {
			swapBuffers();
			m_context.release();
		}

		/**
		 * Redraw the 3D view of the currently selected component. TODO: For now
		 * ALL 3D views are redrawn in the same window. We will have to see if
		 * that is useful.
		 */
		private void redrawGL() {

			// System.out.println("View3D.redrawGL()");

			// String comp = getSelectedComponent();
			// if (comp != null) {
			startRedrawGL();
			try {
				// get draw batches from all components
				Collection values = m_componentConnectors.values();
				for (Iterator i = values.iterator(); i.hasNext();) {
					// GDBPullConnectorOut conn =
					// m_componentConnectors.get(comp);
					GDBPullConnectorOut conn = (GDBPullConnectorOut) i.next();
					DrawBatch batch = conn.pull(new FrameworkQuery(
							getProcessIdentifier(), "Redraw3D"));
					if (batch != null)
						drawBatch3D(batch);
				}
			} catch (FrameworkConnectionException e) {
				System.out.println("Connection to component failed: " + e);
			}
			finishRedrawGL();
			// }
		}

		/**
		 * Draw the 3D contents of a draw batch into the 3D view.
		 */
		private void drawBatch3D(DrawBatch batch) {
			GL gl = m_context.getGL();

			for (int i = 0; i < batch.m_text3Ds.length; i++)
				drawText3D(gl, batch.m_text3Ds[i]);

			for (int i = 0; i < batch.m_point3Ds.length; i++)
				drawPoint3D(gl, batch.m_point3Ds[i]);

			for (int i = 0; i < batch.m_line3Ds.length; i++)
				drawLine3D(gl, batch.m_line3Ds[i]);

			for (int i = 0; i < batch.m_box3Ds.length; i++)
				drawBox3D(gl, batch.m_box3Ds[i]);

			for (int i = 0; i < batch.m_frame3Ds.length; i++)
				drawFrame3D(gl, batch.m_frame3Ds[i]);
		}

		private void drawText3D(GL gl, Text3D text) {
			gl.glColor3d(text.m_color.m_r / 255., text.m_color.m_g / 255.,
					text.m_color.m_b / 255.);
			gl.glRasterPos3d(text.m_x, text.m_y, text.m_z);
			m_glut.glutBitmapString(GLUT.BITMAP_HELVETICA_10, text.m_text);
		}

		private void drawPoint3D(GL gl, Point3D point) {
			gl.glColor3d(point.m_color.m_r / 255., point.m_color.m_g / 255.,
					point.m_color.m_b / 255.);
			if ((point.m_flags & FAT.value) != 0)
				gl.glPointSize(3.f);
			else
				gl.glPointSize(1.f);
			gl.glBegin(GL.GL_POINTS);
			gl.glVertex3d(point.m_x, point.m_y, point.m_z);
			gl.glEnd();
		}

		private void drawLine3D(GL gl, Line3D line) {
			gl.glColor3d(line.m_color.m_r / 255., line.m_color.m_g / 255.,
					line.m_color.m_b / 255.);
			if ((line.m_flags & FAT.value) != 0)
				gl.glLineWidth(3.f);
			else
				gl.glLineWidth(1.f);
			gl.glBegin(GL.GL_LINES);
			gl.glVertex3d(line.m_x1, line.m_y1, line.m_z1);
			gl.glVertex3d(line.m_x2, line.m_y2, line.m_z2);
			gl.glEnd();
		}

		private void drawBox3D(GL gl, Box3D box) {
			gl.glColor3d(box.m_color.m_r / 255., box.m_color.m_g / 255.,
					box.m_color.m_b / 255.);
			if ((box.m_flags & FAT.value) != 0)
				gl.glLineWidth(3.f);
			else
				gl.glLineWidth(1.f);
			gl.glPushMatrix();
			gl.glTranslated(box.m_cx, box.m_cy, box.m_cz);
			gl.glScaled(box.m_sx, box.m_sy, box.m_sz);
			m_glut.glutWireCube(1.f);
			gl.glPopMatrix();
		}

		private void drawFrame3D(GL gl, Frame3D frame) {
			double phi = Math.sqrt(frame.m_rx * frame.m_rx + frame.m_ry
					* frame.m_ry + frame.m_rz * frame.m_rz)
					* 180. / Math.PI;
			gl.glColor3d(frame.m_color.m_r / 255., frame.m_color.m_g / 255.,
					frame.m_color.m_b / 255.);
			gl.glPushMatrix();
			gl.glTranslated(frame.m_px, frame.m_py, frame.m_pz);
			gl.glRotated(phi, frame.m_rx, frame.m_ry, frame.m_rz);
			drawCoordFrame(gl);
			gl.glPopMatrix();
		}

		/**
		 * Draw an arrow pointing in z-axis. Rotate as needed for other axes.
		 * \param axis one of 'x', 'y', 'z' \param index '0', '1', '2', ... '9'
		 */
		private void drawZAxis(GL gl, char axis) {
			double l = 0.10, r = 0.005, a = 0.03, w = 0.020;

			gl.glLineWidth(1.f);
			gl.glBegin(GL.GL_LINES);
			gl.glVertex3d(0., 0., 0.);
			gl.glVertex3d(0., 0., l);
			gl.glEnd();
			gl.glRasterPos3d(0., 0., 1.5 * l);
			m_glut.glutBitmapCharacter(GLUT.BITMAP_HELVETICA_10, axis);
			/*
			 * gluCylinder(glu_quad, r, r, l - a, 10, 1); glPushMatrix();
			 * glTranslated(0., 0., l - a); gluCylinder(glu_quad, w, 0., a, 10,
			 * 1); glPopMatrix(); glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,
			 * col); glRasterPos3d(0., 0., 1.5*l);
			 * glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, axis);
			 * glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, black);
			 */
		}

		/**
		 * Draw the x,y,z coordinate frame.
		 */
		private void drawCoordFrame(GL gl) {
			gl.glPushMatrix();
			drawZAxis(gl, 'z');
			gl.glRotated(-90, 1., 0., 0.);
			drawZAxis(gl, 'y');
			gl.glRotated(90, 0., 1., 0.);
			drawZAxis(gl, 'x');
			gl.glPopMatrix();
		}

		private void drawGroundPlane(GL gl) {
			double grid_size = GROUND_GRID_SIZE;
			int nx = GROUND_NUMDOTS_X;
			int ny = GROUND_NUMDOTS_Y;
			/*
			 * double size_x = GROUND_SIZE_X; double size_y = GROUND_SIZE_Y;
			 * double x, y;
			 */
			double z = GROUND_HEIGHT;

			gl.glNormal3d(0., 0., 1.);
			gl.glLineWidth(1.f);
			gl.glColor3d(1.0, 1.0, 0.0); // yellow
			gl.glBegin(GL.GL_LINES);
			// draw lines parallel y-axis
			for (int i = 0; i < nx; i++) {
				gl.glVertex3d(i * grid_size, 0, z);
				gl.glVertex3d(i * grid_size, (ny - 1) * grid_size, z);
			}
			// draw lines parallel x-axis
			for (int i = 0; i < ny; i++) {
				gl.glVertex3d(0, i * grid_size, z);
				gl.glVertex3d((nx - 1) * grid_size, i * grid_size, z);
			}
			gl.glEnd();
			/*
			 * for (x = 0.; x <= size_x / 2.; x += grid_size) { gl.glVertex3d(x,
			 * -size_y / 2., z); gl.glVertex3d(x, size_y / 2., z); } for (x =
			 * -grid_size; x >= -size_x / 2.; x -= grid_size) { gl.glVertex3d(x,
			 * -size_y / 2., z); gl.glVertex3d(x, size_y / 2., z); } for (y =
			 * 0.; y <= size_y / 2.; y += grid_size) { gl.glVertex3d(-size_x /
			 * 2., y, z); gl.glVertex3d(size_x / 2., y, z); } for (y =
			 * -grid_size; y >= -size_y / 2.; y -= grid_size) {
			 * gl.glVertex3d(-size_x / 2., y, z); gl.glVertex3d(size_x / 2., y,
			 * z); } gl.glBegin(GL.GL_LINE_LOOP); gl.glVertex3d(-size_x / 2.,
			 * -size_y / 2., z); gl.glVertex3d(size_x / 2., -size_y / 2., z);
			 * gl.glVertex3d(size_x / 2., size_y / 2., z); gl.glVertex3d(-size_x /
			 * 2., size_y / 2., z); gl.glEnd();
			 */
			/*
			 * for(x = 0.; x <= size_x/2.; x += grid_size) { snprintf(buf, 100,
			 * "%.1f", x); DrawText3D(buf, x, size_y/2., 0.); } for(y = 0.; y <=
			 * size_y/2.; y += grid_size) { snprintf(buf, 100, "%.1f", y);
			 * DrawText3D(buf, size_x/2., y, 0.); }
			 */
		}
	}

	/**
	 * 
	 */
	private static final String TEXT_TAB = "Output";

	/**
	 * 
	 */
	private static final String ARCH_TAB = "Arch";

	/**
	 * Map of all connections to components. With connection names as keys.
	 */
	private HashMap<String, GDBPullConnectorOut> m_pullConnectors;

	/**
	 * Map of all connections to components. With component names as keys. This
	 * is used to map component names displayed in the list to corresponding
	 * connectors.
	 */
	private HashMap<String, GDBPullConnectorOut> m_componentConnectors;

	// various GUI related stuff
	private View2D m_canvas2;

	private View3D m_canvas3;

	// private List m_connList;

	private StyledText m_text;

	private Group m_controlGroup;

	private Display m_display;

	private Shell m_shell;

	private ArchitectureMonitor m_architectureMonitor;

	private TabFolder m_viewTabs;

	private Table m_processTable;

	private Text m_textOutput;

	private PrintStream m_writer;

	private HashMap<String, ComponentMonitor> m_monitors;

	private Composite m_compOutputComp;

	private ComponentMonitor m_currentlySelected = null;

	private StackLayout m_textOutputStack;

	private SashForm m_rhs;

	private TabItem m_architectureTabItem;

	private MonitorUpdaterThread m_mut;

	private Composite m_lhs;

	private SashForm m_shellForm;

	private ArchitectureView m_architectureView;

	/**
	 * @param _id
	 */
	public UnifiedGUI(String _id) {
		super(_id);

		// System.out.println("********************
		// GraphicalUI.GraphicalUI()");

		m_monitors = new HashMap<String, ComponentMonitor>();
		m_writer = new PrintStream(new GUIOutputStream());

		m_pullConnectors = new HashMap<String, GDBPullConnectorOut>();
		m_componentConnectors = new HashMap<String, GDBPullConnectorOut>();
	}

	/**
	 * Returns whether the given component is selected at the moment.
	 */
	private boolean componentIsSelected(String compID) {
		if (m_processTable.getSelectionCount() > 0) {
			String selectedID = m_processTable.getSelection()[0].getText();
			return compID.equals(selectedID);
		} else {
			return false;
		}
	}

	/**
	 * Draw the Text contents of a draw batch into the text view.
	 */
	private void drawBatchText(DrawBatch batch) {
		m_text.setText("");
		for (int i = 0; i < batch.m_texts.length; i++)
			m_text.append(batch.m_texts[i]);
	}

	/**
	 * Returns the single selected component or null if no or several components
	 * are selected.
	 */
	private String getSelectedComponent() {

		if (m_processTable.getSelectionCount() > 0) {
			return m_processTable.getSelection()[0].getText();
		} else {
			return null;
		}

	}

	/**
	 * Returns the selected view: "Architecture", "Output", "2D", "3D" or
	 * "Text".
	 */
	private String getSelectedView() {

		try {
			TabItem view = m_viewTabs.getItem(m_viewTabs.getSelectionIndex());
			return view.getText();
		} catch (SWTException e) {
			// e.printStackTrace();
			System.out.println("Exception in getSelectedViews");
			System.out.println(e.getLocalizedMessage());
		}

		return "";

	}

	/**
	 * receivePushData called asyncexec to notify an explicit redraw request for
	 * a given component. Check if that component is currently selected and if
	 * so redraw.
	 */
	private void handleExplicitRedraw(String component) {
		if (componentIsSelected(component)) {
			redrawAllViews();
		}
	}

	private void linkMonitorsToGUI() {
		boolean first = true;
		for (ComponentMonitor monitor : m_monitors.values()) {
			monitor.setOutStream(m_writer);
			monitor.setOutputArea(m_compOutputComp);
			monitor.setDisplay(m_display);

			if (first) {
				m_currentlySelected = monitor;
				// m_currentlySelected.textAreaVisible(true);
				first = false;
				m_textOutputStack.topControl = m_currentlySelected
						.getOutputComposite();
				m_compOutputComp.layout();
			}
		}

		// ScrolledComposite comp = new ScrolledComposite(m_viewTabs,
		// SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);

		// Composite comp = new Composite(m_viewTabs, SWT.BORDER
		// // | SWT.V_SCROLL | SWT.H_SCROLL);
		// );
		//
		// comp.setLayout(new FillLayout());

		// m_architectureView =
		// new ArchitectureView(m_architectureMonitor, m_display,
		// m_viewTabs);
		// m_architectureTabItem.setControl(m_architectureView.getView());
	}

	/**
	 * Redraw the Text view of the currently selected component. Note that this
	 * is only necessary for an explicit redraw. Normally the text widget takes
	 * care of redrawing itself.
	 */
	private void redrawText() {
		String comp = getSelectedComponent();
		if (comp != null) {
			try {
				GDBPullConnectorOut conn = m_componentConnectors.get(comp);
				DrawBatch batch = conn.pull(new FrameworkQuery(
						getProcessIdentifier(), "RedrawText"));
				drawBatchText(batch);
			} catch (FrameworkConnectionException e) {
				System.out.println("Connection to component failed: " + e);
			}
		}
	}

	/**
	 * Explicitely redraw views. This is typically triggered by a redraw request
	 * from a component. Note that at the moment we only draw one (the selected)
	 * view.
	 */
	private void redrawAllViews() {
		String view = getSelectedView();
		// find which view is active and pull the according information
		if (view.equals("2D")) {
			// m_canvas2.redraw();
			m_canvas2.redrawGL();
		} else if (view.equals("3D")) {
			// m_canvas3.redraw();
			m_canvas3.redrawGL();
		} else if (view.equals("Text")) {
			redrawText();
		}

	}

	/**
	 * Redraw the text view. This is required whenever we switch to the text
	 * view. (The 2D and 3D view handle that automatically via their paint
	 * methods).
	 */
	private void redrawTextView() {
		String view = getSelectedView();
		if (view.equals("Text")) {
			redrawText();
		}
	}

	/**
	 * @param _comp
	 */
	private void setupConnectionList(Composite _comp) {

		// m_processTable = new Table (_comp, SWT.BORDER | SWT.V_SCROLL
		// |
		// SWT.H_SCROLL);
		// 
		// m_processTable.setLinesVisible (true);
		// for (int i=0; i<3; i++) {
		// TableColumn column = new TableColumn(m_processTable,
		// SWT.NONE);
		// column.setWidth (100);
		// }
		//        
		// for (int i=0; i<12; i++) {
		// new TableItem (m_processTable, SWT.NONE);
		// }

		// TableItem [] items = m_processTable.getItems ();
		// for (int i=0; i<items.length; i++) {
		// TableEditor editor = new TableEditor (m_processTable);
		// CCombo combo = new CCombo (m_processTable, SWT.NONE);
		// combo.setText("CCombo");
		// combo.add("item 1");
		// combo.add("item 2");
		// editor.grabHorizontal = true;
		// editor.setEditor(combo, items[i], 0);
		// editor = new TableEditor (m_processTable);
		// Text text = new Text (m_processTable, SWT.NONE);
		// text.setText("Text");
		// editor.grabHorizontal = true;
		// editor.setEditor(text, items[i], 1);
		// editor = new TableEditor (m_processTable);
		// Button button = new Button (m_processTable, SWT.CHECK);
		// button.pack ();
		// editor.minimumWidth = button.getSize ().x;
		// editor.horizontalAlignment = SWT.LEFT;
		// editor.setEditor (button, items[i], 2);
		// }

		m_processTable = new Table(_comp, SWT.BORDER | SWT.SINGLE
				| SWT.NO_REDRAW_RESIZE);

		GridData data = new GridData(SWT.FILL, SWT.FILL, true, true);
		// RowData data = new RowData();

		m_processTable.setLayoutData(data);

		m_processTable.setLinesVisible(true);
		m_processTable.setHeaderVisible(true);

		TableColumn column1 = new TableColumn(m_processTable, SWT.CENTER);
		TableColumn column2 = new TableColumn(m_processTable, SWT.CENTER);
		TableColumn column3 = new TableColumn(m_processTable, SWT.CENTER);
		TableColumn column4 = new TableColumn(m_processTable, SWT.CENTER);

		// column1.
		// column1.setWidth(200);
		column1.setText("name");
		column2.setText("P");
		column3.setText("L");
		column4.setText("D");

		column1.setResizable(true);
		column2.setResizable(false);
		column3.setResizable(false);
		column4.setResizable(false);

		column2.setWidth(30);
		column3.setWidth(30);
		column4.setWidth(30);

		// if (m_architectureMonitor.getMotiveGenerator() != null) {
		// m_architectureMonitor.getMotiveGenerator().getTableItem(
		// m_processTable);
		// m_architectureMonitor.getMotiveManager().getTableItem(
		// m_processTable);
		// }

		for (SubarchitectureMonitor subarch : m_architectureMonitor) {

			// TableItem saitem = subarch.getTableItem(m_processTable);

			subarch.getWorkingMemory().getTableItem(m_processTable);
			subarch.getTaskManager().getTableItem(m_processTable);

			for (ComponentMonitor monitor : subarch.getDataDrivenComponents()) {
				monitor.getTableItem(m_processTable);
			}

			for (ComponentMonitor monitor : subarch.getGoalDrivenComponents()) {
				monitor.getTableItem(m_processTable);
			}

			// saitem.setExpanded(true);
			// saitem.get
		}

		column1.pack();
		// column2.pack();
		// column3.pack();
		// column4.pack();

		m_processTable.setSelection(m_processTable.getItem(0));

		m_processTable.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent event) {
				try {
					componentSelectionChanged();
				} catch (SWTException e) {
					// e.printStackTrace();
					System.out.println("Exception in widgetDefaultSelected");
					System.out.println(e.getLocalizedMessage());
				}
			}

			public void widgetSelected(SelectionEvent event) {
				// Note: event.item.toString() returns something like
				// 'TabItem {2D}', which
				// is rather unsuited. Therefore just inform the main
				// class and let it find
				// out what the current selection is.
				try {
					componentSelectionChanged();
				} catch (SWTException e) {
					// e.printStackTrace();
					System.out.println("Exception in widgetSelected");
					System.out.println(e.getLocalizedMessage());
				}
			}
		});

		m_processTable.addListener(SWT.Selection, new Listener() {

			public void handleEvent(Event e) {
				TableItem[] selection = m_processTable.getSelection();
				ComponentMonitor monitor = m_monitors.get(selection[0]
						.getText());
				// System.out.println("selected = \"" +
				// selection[0].getText() + "\"");
				if (monitor != null) {
					m_currentlySelected = monitor;
					m_textOutputStack.topControl = m_currentlySelected
							.getOutputComposite();
					m_compOutputComp.layout();
					m_compOutputComp.redraw();
					// m_currentlySelected.textAreaVisible(false);
					// m_currentlySelected = monitor;
					// m_currentlySelected.textAreaVisible(true);
				}
			}
		});

		// TreeItem[] items = m_processTree.getItems();
		// TreeEditor editor;
		// for (int i = 0; i < items.length; i++) {
		// editor = new TreeEditor(m_processTree);
		// editor.minimumHeight = 50;
		// editor.setEditor(button, saitem, 3);
		// }

		for (Iterator<String> it = m_pullConnectors.keySet().iterator(); it
				.hasNext();) {
			// fill list with component names
			try {
				GDBPullConnectorOut conn = m_pullConnectors.get(it.next());
				DrawBatch batch = conn.pull(new FrameworkQuery(
						getProcessIdentifier(), "GetConfig"));
				// store component name vs. connector
				m_componentConnectors.put(batch.m_compID, conn);
			} catch (FrameworkConnectionException e) {
			}
		}

		m_processTable.pack(true);
	}

	private void setupControls(Composite parent) {

		parent.setLayout(new GridLayout(1, true));

		m_controlGroup = new Group(parent, SWT.SHADOW_ETCHED_IN | SWT.V_SCROLL
				| SWT.H_SCROLL);
		// m_controlGroup.setLayout(new RowLayout(SWT.VERTICAL));

		m_controlGroup.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true,
				true));

		m_controlGroup.setLayout(new GridLayout(1, true));
		m_controlGroup.setText("Components");
		setupConnectionList(m_controlGroup);

		m_controlGroup.pack(true);
	}

	/**
	 * @param _rhs
	 */
	private void setupOutputArea(Composite _rhs) {
		m_textOutput = new Text(_rhs, SWT.BORDER | SWT.V_SCROLL);
		m_textOutput.setText("\n\n\n\n\n\n\n"); // hack to determine
		// number of lines
		// initially visible
		m_textOutput.setEditable(false);
		// m_textOutput.re
		// t.set
		// GridData data = new GridData(GridData.FILL, GridData.CENTER,
		// false,
		// false);
		// // GridData data = new GridData(GridData.FILL_HORIZONTAL);
		// m_textOutput.setLayoutData(data);
		// m_textOutput.setSize(600,600);

	}

	/**
	 * Create the text view.
	 */
	private void setupText(Composite parent) {
		m_text = new StyledText(parent, SWT.MULTI | SWT.BORDER | SWT.READ_ONLY);
	}

	/**
	 * Create the tab folder with views on the right.
	 */
	private void setupViews(Composite _parent) {
		m_viewTabs = new TabFolder(_parent, SWT.BOTTOM | SWT.H_SCROLL
				| SWT.V_SCROLL);

		// RowData data = new RowData(SWT.FILL, SWT.FILL, true, true);
		// m_viewTabs.setLayoutData(data);

		// m_architectureTabItem = new TabItem(m_viewTabs, SWT.NONE);
		// m_architectureTabItem.setText(UnifiedGUI.ARCH_TAB);
		TabItem item;

		item = new TabItem(m_viewTabs, SWT.NONE);
		item.setText("2D");
		Composite comp2 = new Composite(m_viewTabs, SWT.NONE);
		comp2.setLayout(new FillLayout());
		GLData data = new GLData();
		data.doubleBuffer = true;
		m_canvas2 = new View2D(comp2, data);
		item.setControl(comp2);

		item = new TabItem(m_viewTabs, SWT.NONE);
		item.setText("3D");
		GLData data2 = new GLData();
		data2.doubleBuffer = true;
		m_canvas3 = new View3D(m_viewTabs, data2);
		item.setControl(m_canvas3);

		item = new TabItem(m_viewTabs, SWT.NONE);
		item.setText("Text");
		setupText(m_viewTabs);
		item.setControl(m_text);

		TabItem textItem = new TabItem(m_viewTabs, SWT.NONE);
		textItem.setText(UnifiedGUI.TEXT_TAB);

		m_compOutputComp = new Composite(m_viewTabs, SWT.NO_REDRAW_RESIZE);

		m_textOutputStack = new StackLayout();

		m_compOutputComp.setLayout(m_textOutputStack);

		textItem.setControl(m_compOutputComp);

		// inform the GUI shen selection changed
		m_viewTabs.addSelectionListener(new SelectionListener() {

			public void widgetDefaultSelected(SelectionEvent event) {
				try {
					viewSelectionChanged();
				} catch (SWTException e) {
					// e.printStackTrace();
					System.out.println("Exception in widgetDefaultSelected");
					System.out.println(e.getLocalizedMessage());
				}
			}

			public void widgetSelected(SelectionEvent event) {
				// Note: event.item.toString() returns something like
				// 'TabItem {2D}', which
				// is rather unsuited. Therefore just inform the main
				// class and let it find
				// out what the current selection is.
				try {
					viewSelectionChanged();
				} catch (SWTException e) {
					// e.printStackTrace();
					System.out.println("Exception in widgetSelected");
					System.out.println(e.getLocalizedMessage());
				}
			}
		});
	}

	/**
	 * Create and layout all our widgets.
	 */
	private void setupWidgets() {

		// create display and top level window (shell)
		m_display = Display.getDefault();

		// if the previous statement didn't create the display, then
		// we're in a lot of trouble!
		if (null == Display.getCurrent()) {
			throw new RuntimeException(
					"Main GUI thread is not current interface thread!");
		}

		m_shell = new Shell(m_display);
		m_shell.setText("CAST Architecture GUI");

		m_shell.setLayout(new FillLayout());
		m_shellForm = new SashForm(m_shell, SWT.HORIZONTAL | SWT.BORDER);

		m_lhs = new Composite(m_shellForm, SWT.NONE);

		setupControls(m_lhs);

		m_rhs = new SashForm(m_shellForm, SWT.VERTICAL | SWT.BORDER);
		m_rhs.SASH_WIDTH = 10;

		setupViews(m_rhs);
		setupOutputArea(m_rhs);

		m_rhs.setWeights(new int[] { 70, 30 });
		m_shellForm.setWeights(new int[] { 30, 70 });

		m_shell.pack();
		m_shell.setSize(1000, 800);
		m_shell.open();
	}

	protected void append(String _s) {
		m_display.syncExec(new AppendRunnable(_s, m_textOutput));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.ui.architecture.UIServer#handleComponentEvent(java.lang.String,
	 *      caat.corba.autogen.CAAT.ui.ComponentEvent)
	 */
	@Override
	protected void handleComponentEvent(String _src, ComponentEvent _data) {
		ComponentMonitor monitor = m_monitors.get(_src);
		if (monitor != null) {
			monitor.componentEvent(_data);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.ui.architecture.UIServer#handleTextOutput(java.lang.String,
	 *      caat.corba.autogen.CAAT.ui.TextOutput)
	 */
	@Override
	protected void handleTextOutput(String _src, TextOutput _data) {
		ComponentMonitor monitor = m_monitors.get(_src);
		if (monitor != null) {
			monitor.outputText(_data);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.ui.architecture.UIServer#setup(caat.configuration.ArchitectureConfiguration)
	 */
	@Override
	protected void setArchitectureConfiguration(
			ArchitectureConfiguration _connectionConfiguration) {

		m_architectureMonitor = new ArchitectureMonitor(
				_connectionConfiguration, m_writer);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.ui.architecture.UIServer#setComponentConnection(caat.corba.autogen.CAAT.ui.ComponentStatus,
	 *      caat.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnectorOut)
	 */
	@Override
	protected void setupComponentConnection(ComponentStatus _componentStatus,
			ComponentStatusPullConnectorOut _conn) {

		ComponentMonitor monitor = null;
		try {
			// System.out.println("setupCC: " + _logObject.m_component);
			monitor = m_architectureMonitor.getComponent(
					_componentStatus.m_component,
					_componentStatus.m_subarchitecture);
			// System.out.println("setupCC: " + monitor.getID());
		} catch (CASTUIException e) {
			e.printStackTrace();
			System.exit(1);
		}

		monitor.setConnection(_conn);

		m_monitors.put(monitor.getID(), monitor);
		// monitor.setInitialValues(_logObject);
	}

	/**
	 * Also a change in the component selection makes a redraw necessary.
	 */
	public void componentSelectionChanged() {

		redrawAllViews();

	}

	/**
	 * Receiving a push from a component means that component wants to be
	 * redrawn (issues an explicit redraw request).
	 */
	public void receivePushData(final String _src, DrawBatch _data) {

		try {
			m_display.asyncExec(new Runnable() {

				public void run() {
					handleExplicitRedraw(_src);
				}
			});
		} catch (SWTException e) {
			// e.printStackTrace();
			System.out.println("Exception in redraw runnable");
			System.out.println(e.getLocalizedMessage());
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.processes.FrameworkProcess#run()
	 */
	@Override
	public void run() {
		// maybe we should update the GUI regularly?
		final int updateRate = 500; // in [ms]

		Runnable timer = new TimedUpdateRunnable(updateRate);

		setupWidgets();
		linkMonitorsToGUI();
		m_shell.open();
		m_display.timerExec(updateRate, timer);
		while (!m_shell.isDisposed()) {
			try {
				if (!m_display.readAndDispatch()) {
					m_display.sleep();
				}
			} catch (SWTException e) {
				// e.printStackTrace();
				System.out.println("Exception in GUI event loop");
				System.out.println(e.getLocalizedMessage());
			}

		}
		m_display.dispose();
	}

	public void setPullConnector(String _connectionID,
			GDBPullConnectorOut _senderAdaptor) {
		m_pullConnectors.put(_connectionID, _senderAdaptor);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.ui.architecture.UIServer#start()
	 */
	@Override
	public void start() {
		super.start();

		// m_mut = new MonitorUpdaterThread();
		// m_mut.start();

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.processes.FrameworkProcess#stop()
	 */
	@Override
	public void stop() {
		super.stop();
		// m_mut.stopUpdating();
		// try {
		// m_mut.join();
		// }
		// catch (InterruptedException e) {
		// e.printStackTrace();
		// System.exit(1);
		// }
	}

	/**
	 * Also a change in the view selection makes a redraw necessary.
	 */
	public void viewSelectionChanged() {
		redrawTextView();
	}

	private boolean scheduleUpdate(final int _updateRate, Runnable _r) {
		try {
			m_display.timerExec(_updateRate, _r);
			return true;
		} catch (SWTException e) {
			System.out.println("Exception in GUI runnable");
			System.out.println(e.getLocalizedMessage());
		}
		return false;
	}
}
