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
package cast.ui.inspectable;

import java.nio.ByteBuffer;
import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import javax.media.opengl.*;
import javax.media.opengl.glu.GLU;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.layout.*;
import org.eclipse.swt.opengl.GLCanvas;
import org.eclipse.swt.opengl.GLData;
import org.eclipse.swt.widgets.*;
import org.eclipse.swt.widgets.List;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.data.FrameworkQuery;
import balt.core.processes.FrameworkProcess;
import cast.cdl.guitypes.*;
import cast.ui.UIUtils;
import cast.ui.inspectable.GDBPullInterface.GDBPullConnectorOut;
import cast.ui.inspectable.GDBPullInterface.GDBPullSender;
import cast.ui.inspectable.GDBPushInterface.GDBPushReceiver;

/**
 * A simple graphical user interface to visualise all the components running
 * in a system.
 *
 * TODO: - store/restore PROJECTION_MATRIX for each view
 *       - add level of detail to redraw query
 *       - add Log query: text is remembered and scrolled off screen
 *
 * @author mxz
 */
public class GUI extends FrameworkProcess
		implements GDBPullSender, GDBPushReceiver
{
	/**
	 * Init datatypes used for connections.
	 */
	static {
		UIUtils.init();
	}

	/**
	 * Map of all connections to components.
	 * With connection names as keys.
	 */
	private HashMap<String, GDBPullConnectorOut> m_pullConnectors;

	/**
	 * Map of all connections to components.
	 * With component names as keys. This is used to map component names displayed in the
	 * list to corresponding connectors. 
	 */
	private HashMap<String, GDBPullConnectorOut> m_componentConnectors;

	/*
	 * GL viewing parameters for the various views for each component
	 */
	private HashMap<String, double[]> m_projMat2D;
	private HashMap<String, double[]> m_modelviewMat2D;
	private HashMap<String, double[]> m_projMat3D;
	private HashMap<String, double[]> m_modelviewMat3D;

	/**
	 * indicates that a visible component requested a redraw
	 */
	private String m_needsRedraw = "";
	private final Lock m_lock = new ReentrantLock();

	// various GUI related stuff
	private Display m_display;
	private Shell m_shell;
	private GLCanvas m_canvas2;
	private GLContext m_context2;
	private GL m_gl2;
	private GLU m_glu2;
	private GLCanvas m_canvas3;
	private GLContext m_context3;
	private GL m_gl3;
	private GLU m_glu3;
	private List m_connList;
	private Text m_text;
	private TabFolder m_viewTabs;
	private Group m_controlGroup;


	public GUI(String _id)
	{
		super(_id);
		m_pullConnectors = new HashMap<String, GDBPullConnectorOut>();
		m_componentConnectors = new HashMap<String, GDBPullConnectorOut>();
		m_projMat2D = new HashMap<String, double[]>();
		m_projMat3D = new HashMap<String, double[]>();
		m_modelviewMat2D = new HashMap<String, double[]>();
		m_modelviewMat3D = new HashMap<String, double[]>();
	}

	/**
	 * Receiving a push from a component means that component wants to be
	 * redrawn (issues an explicit redraw request).
	 */
	public void receivePushData(String _src, DrawBatch _data)
	{
		m_lock.lock();
		// remember which component issued the redraw request
		m_needsRedraw = _src;
		m_lock.unlock();
		m_display.asyncExec(null);
	}

	public void setPullConnector(String _connectionID,
			GDBPullConnectorOut _senderAdaptor)
	{
		m_pullConnectors.put(_connectionID, _senderAdaptor);
	}

	/**
	 * receivePushData called asyncexec to notify an explicit redraw
	 * request for a given component.
	 * Check if that component is currently selected and if so redraw.
	 */
	private void handleExplicitRedraw()
	{
		boolean redraw = false;
		m_lock.lock();
		// TODO: using componentIsSelected() mysteriously causes freezes
		// possible reason: not using the selected check means that every redraw
		// request from any component (selected or not) will cause a redraw of the
		// currently selected component. seemingly components sometimes just freeze (stop
		// sending redraw requests). if some other (not selected!) component can also cause
		// a redraw, this freeze becomes not apparent.
		if(m_needsRedraw != "" && componentIsSelected(m_needsRedraw))
			redraw = true;
		m_needsRedraw = "";
		m_lock.unlock();
		if(redraw)
			redrawViews();
	}

	/**
	 * Check and act upon any pending GUI events.
	 */
	private void handleEvents()
	{
		while(m_display.readAndDispatch())
			;
	}

	/**
	 * Also a change in the view selection makes a redraw necessary.
	 */
	public void viewSelectionChanged()
	{
		redrawViews();
	}

	/**
	 * Also a change in the component selection makes a redraw necessary.
	 */
	public void componentSelectionChanged()
	{
		redrawViews();
	}

	/**
	 * Returns whether the given component is selected at the moment. 
	 */
	private boolean componentIsSelected(String compID)
	{
		int idx = m_connList.indexOf(compID);
		if(idx >= 0)
			return m_connList.isSelected(idx);
		else
			return false;
	}

	/**
	 * Returns the single selected component or null if no or several
	 * components are selected.
	 */
	private String getSelectedComponent()
	{
		String sel[] = m_connList.getSelection();
		if(sel.length == 1)
			return sel[0];
		else
			return null;
	}

	/**
	 * Returns the selected view: "2D", "3D" or "Text".
	 */
	private String getSelectedView()
	{
		TabItem view = m_viewTabs.getItem(m_viewTabs.getSelectionIndex()); 
		return view.getText();
	}

	/**
	 * Create and layout all our widgets.
	 */
	private void setupWidgets()
	{
		// create display and top level window (shell)
		m_display = new Display();
		m_shell = new Shell(m_display);
		m_shell.setText("CAAT GUI");
		GridLayout shellLayout = new GridLayout(2, false);
		m_shell.setLayout(shellLayout);

		// create all our various widgets
		setupControls(m_shell);

        setupViews(m_shell);

		// do the layout details for all the little widgets
		GridData data = new GridData(SWT.FILL, SWT.FILL, false, true);
		m_controlGroup.setLayoutData(data);
		data = new GridData(SWT.FILL, SWT.FILL, true, true);
		m_viewTabs.setLayoutData(data);
	}

	/**
	 * Create the box of control thingies on the left
	 */
	private void setupControls(Composite parent)
	{
		m_controlGroup = new Group(parent, SWT.SHADOW_ETCHED_IN);
		m_controlGroup.setLayout(new RowLayout(SWT.VERTICAL));
		m_controlGroup.setText("Components");

		setupConnectionList(m_controlGroup);
	}

	/**
	 * TODO: provide a button "None" to deselect all.
	 */
	private void setupConnectionList(Composite parent)
	{
		m_connList = new org.eclipse.swt.widgets.List(parent, SWT.SINGLE);
		for(Iterator<String> it = m_pullConnectors.keySet().iterator(); it.hasNext(); )
		{
			// fill list with component names 
			try
			{
				GDBPullConnectorOut conn = m_pullConnectors.get(it.next());
				DrawBatch batch = conn.pull(new FrameworkQuery(getProcessIdentifier(), "GetConfig"));
				// store component name vs. connector
				m_componentConnectors.put(batch.m_compID, conn);
				// and store component name in list
				m_connList.add(batch.m_compID);
				// prepare viewing parameters
				m_projMat2D.put(batch.m_compID, new double[16]);
				m_modelviewMat2D.put(batch.m_compID, new double[16]);
				m_projMat3D.put(batch.m_compID, new double[16]);
				m_modelviewMat3D.put(batch.m_compID, new double[16]);
			}
			catch(FrameworkConnectionException e)
			{
				m_connList.add("ERROR");
			}
		}

		// inform the GUI when selection changed
		m_connList.addSelectionListener(new SelectionListener()
		{
			public void widgetSelected(SelectionEvent event)
			{
				// Note: event.item.toString() returns something like 'TabItem {2D}', which
				// is rather unsuited. Therefore just inform the main class and let it find
				// out what the current selection is.
				componentSelectionChanged();
			}
			public void widgetDefaultSelected(SelectionEvent event)
			{
				componentSelectionChanged();
			}
		});
	}

	/**
	 * Create the tab folder with views on the right.
	 */
	private void setupViews(Composite parent)
	{
		m_viewTabs = new TabFolder(parent, SWT.BOTTOM);

		TabItem item = new TabItem(m_viewTabs, SWT.NONE);
		item.setText("2D");
		setupGL2D(m_viewTabs);
		item.setControl(m_canvas2);
//
//		item = new TabItem(m_viewTabs, SWT.NONE);
//		item.setText("3D");
//		setupGL3D(m_viewTabs);
//		item.setControl(m_canvas3);

        item = new TabItem(m_viewTabs, SWT.NONE);
		item.setText("Text");
		setupText(m_viewTabs);
		item.setControl(m_text);

		// inform the GUI shen selection changed
		m_viewTabs.addSelectionListener(new SelectionListener()
		{
			public void widgetSelected(SelectionEvent event)
			{
				// Note: event.item.toString() returns something like 'TabItem {2D}', which
				// is rather unsuited. Therefore just inform the main class and let it find
				// out what the current selection is.
				viewSelectionChanged();
			}
			public void widgetDefaultSelected(SelectionEvent event)
			{
				viewSelectionChanged();
			}
		});
	}

	/**
	 * Create and properly initialise the 2D GL canvas.
	 */
	private void setupGL2D(Composite parent)
	{
		GLData data = new GLData();
		data.doubleBuffer = true;

		m_canvas2 = new GLCanvas(parent, SWT.NONE, data);
		m_canvas2.setCurrent();

		m_context2 = GLDrawableFactory.getFactory().createExternalGLContext();
		m_glu2 = new GLU();

		m_context2.makeCurrent();
		m_gl2 = m_context2.getGL();
		m_gl2.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		m_gl2.glDisable(GL.GL_DEPTH_TEST);
		m_gl2.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1);
		m_gl2.glPixelZoom(1.0f, -1.0f);
		m_context2.release();

		m_canvas2.addListener(SWT.Resize, new Listener()
		{
			public void handleEvent(Event event)
			{
				resizeGL2D();
			}
		});

		m_canvas2.addListener(SWT.Paint, new Listener()
		{
			public void handleEvent(Event event)
			{
				redraw2D();
			}
		});
	}

	/**
	 * Create and properly initialise the 2D GL canvas.
	 */
	private void setupGL3D(Composite parent)
	{
		GLData data = new GLData();
		data.doubleBuffer = true;

		m_canvas3 = new GLCanvas(parent, SWT.NONE, data);
		m_canvas3.setCurrent();

		m_context3 = GLDrawableFactory.getFactory().createExternalGLContext();
		m_glu3 = new GLU();

		m_context3.makeCurrent();
		m_gl3 = m_context3.getGL();
		m_gl3.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		m_gl3.glEnable(GL.GL_DEPTH_TEST);
		m_context3.release();

		m_canvas3.addListener(SWT.Resize, new Listener()
		{
			public void handleEvent(Event event)
			{
				resizeGL3D();
			}
		});

		m_canvas3.addListener(SWT.Paint, new Listener()
		{
			public void handleEvent(Event event)
			{
				redraw3D();
			}
		});
	}

	/**
	 * Create the text view.
	 */
	private void setupText(Composite parent)
	{
		m_text = new Text(parent, SWT.MULTI | SWT.BORDER | SWT.READ_ONLY);
	}

	private void resizeGL2D()
	{
		m_canvas2.setCurrent();
		m_context2.makeCurrent();
		Rectangle bounds = m_canvas2.getBounds();

		m_gl2.glViewport(0, 0, bounds.width, bounds.height);
		m_gl2.glMatrixMode(GL.GL_PROJECTION);
		m_gl2.glLoadIdentity();
		m_gl2.glScaled(1.0, -1.0, 1.0);  // y points down
		m_glu2.gluOrtho2D(0, bounds.width, 0, bounds.height);
		m_gl2.glMatrixMode(GL.GL_MODELVIEW);
		m_gl2.glLoadIdentity();

		m_context2.release();

		redraw2D();
	}

	private void resizeGL3D()
	{
		m_canvas3.setCurrent();
		m_context3.makeCurrent();
		Rectangle bounds = m_canvas3.getBounds();

		m_gl3.glViewport(0, 0, bounds.width, bounds.height);
		m_gl3.glMatrixMode(GL.GL_PROJECTION);
		m_gl3.glLoadIdentity();
		// viewing frustrum, note all values are in m.
		m_glu3.gluPerspective(60., bounds.width/bounds.height, 0.001, 100.);
		m_glu3.gluLookAt(1.0, 2.0, 1.0,  0.0, 0.0, 0.0,  0., 0., 1.0);
		m_gl3.glMatrixMode(GL.GL_MODELVIEW);
		m_gl3.glLoadIdentity();

		m_context3.release();

		redraw3D();
	}

	/**
	 * Redraw the 2D view of the currently selected component.
	 * Triggered by expose, resize or explicit redraw.
	 */
	private void redraw2D()
	{
		String comp = getSelectedComponent();
		if(comp != null)
		{
			try
			{
				GDBPullConnectorOut conn = m_componentConnectors.get(comp);
				DrawBatch batch = conn.pull(new FrameworkQuery(getProcessIdentifier(), "Redraw2D"));
				drawBatch2D(batch);
			}
			catch(FrameworkConnectionException e)
			{
				System.out.println("Connection to component failed: " + e);
			}
		}
	}

	/**
	 * Redraw the 3D view of the currently selected component.
	 * Triggered by expose, resize or explicit redraw.
	 */
	private void redraw3D()
	{
		String comp = getSelectedComponent();
		if(comp != null)
		{
			try
			{
				GDBPullConnectorOut conn = m_componentConnectors.get(comp);
				DrawBatch batch = conn.pull(new FrameworkQuery(getProcessIdentifier(), "Redraw3D"));
				drawBatch3D(batch);
			}
			catch(FrameworkConnectionException e)
			{
				System.out.println("Connection to component failed: " + e);
			}
		}
	}

	/**
	 * Redraw the Text view of the currently selected component.
	 * Note that this is only necessary for an explicit redraw. Normally the text widget
	 * takes care of redrawing itself.
	 */
	private void redrawText()
	{
		String comp = getSelectedComponent();
		if(comp != null)
		{
			try
			{
				GDBPullConnectorOut conn = m_componentConnectors.get(comp);
				DrawBatch batch = conn.pull(new FrameworkQuery(getProcessIdentifier(), "RedrawText"));
				drawBatchText(batch);
			}
			catch(FrameworkConnectionException e)
			{
				System.out.println("Connection to component failed: " + e);
			}
		}
	}

	/**
	 * Explicitely redraw views. This is typically triggered by a redraw request from
	 * a component,
	 * Note that at the moment we only draw one (the selected) view.
	 */
  private void redrawViews()
  {
		String view = getSelectedView(); 
		// find which view is active and pull the according information
		if(view == "2D")
			redraw2D();
		else if(view == "3D")
			redraw3D();
		else if(view == "Text")
			redrawText();
  }

  /**
   * Draw the 2D contents of a draw batch into the 2D view.
   */
	private void drawBatch2D(DrawBatch batch)
	{
		m_canvas2.setCurrent();
		m_context2.makeCurrent();

		m_gl2.glClear(GL.GL_COLOR_BUFFER_BIT);
		m_gl2.glMatrixMode(GL.GL_MODELVIEW);
		m_gl2.glLoadIdentity();

		// the image is drawn first, so it acts a background, all other drawing
		// primitives being overlaid
		if(batch.m_image != null && batch.m_image.m_rgbBuffer.length != 0)
			drawRGBImage(batch.m_image);

		for(int i = 0; i < batch.m_point2Ds.length; i++)
			drawPoint2D(batch.m_point2Ds[i]);

		for(int i = 0; i < batch.m_line2Ds.length; i++)
			drawLine2D(batch.m_line2Ds[i]);

		for(int i = 0; i < batch.m_rect2Ds.length; i++)
			drawRect2D(batch.m_rect2Ds[i]);

		for(int i = 0; i < batch.m_poly2Ds.length; i++)
			drawPolygon2D(batch.m_poly2Ds[i]);

		m_canvas2.swapBuffers();
		m_context2.release();
	}

  /**
   * Draw the 3D contents of a draw batch into the 2D view.
   */
	private void drawBatch3D(DrawBatch batch)
	{
		m_canvas3.setCurrent();
		m_context3.makeCurrent();

		m_gl3.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
		m_gl3.glMatrixMode(GL.GL_MODELVIEW);
		m_gl3.glLoadIdentity();

		for(int i = 0; i < batch.m_point3Ds.length; i++)
			drawPoint3D(batch.m_point3Ds[i]);

		for(int i = 0; i < batch.m_line3Ds.length; i++)
			drawLine3D(batch.m_line3Ds[i]);

		m_canvas3.swapBuffers();
		m_context3.release();
	}

  /**
   * Draw the Text contents of a draw batch into the text view.
   */
	private void drawBatchText(DrawBatch batch)
	{
		m_text.setText("");
		for(int i = 0; i < batch.m_texts.length; i++)
			m_text.append(batch.m_texts[i]);
	}

	private void drawRGBImage(RGBImage img)
	{
		ByteBuffer bb = ByteBuffer.allocate(img.m_rgbBuffer.length);
		bb.put(img.m_rgbBuffer);
		bb.position(0);
		bb.limit(bb.capacity());
		m_gl2.glRasterPos2d(0., 0.);
		m_gl2.glDrawPixels(img.m_width, img.m_height,
				GL.GL_RGB, GL.GL_UNSIGNED_BYTE, bb);
	}

	private void drawPoint2D(Point2D point)
	{
		m_gl2.glColor3d(point.m_color.m_r/255., point.m_color.m_g/255., point.m_color.m_b/255.);
		if((point.m_flags & FAT.value) != 0)
			m_gl2.glPointSize(3.f);
		else
			m_gl2.glPointSize(1.f);
		m_gl2.glBegin(GL.GL_POINTS);
		m_gl2.glVertex2d(point.m_x, point.m_y);
		m_gl2.glEnd();
	}

	private void drawLine2D(Line2D line)
	{
		m_gl2.glColor3d(line.m_color.m_r/255., line.m_color.m_g/255., line.m_color.m_b/255.);
		if((line.m_flags & FAT.value) != 0)
			m_gl2.glLineWidth(3.f);
		else
			m_gl2.glLineWidth(1.f);
		m_gl2.glBegin(GL.GL_LINES);
		m_gl2.glVertex2d(line.m_x1, line.m_y1);
		m_gl2.glVertex2d(line.m_x2, line.m_y2);
		m_gl2.glEnd();		
	}

	private void drawRect2D(Rect2D rect)
	{
		m_gl2.glColor3d(rect.m_color.m_r/255., rect.m_color.m_g/255., rect.m_color.m_b/255.);
		if((rect.m_flags & FAT.value) != 0)
			m_gl2.glLineWidth(3.f);
		else
			m_gl2.glLineWidth(1.f);
		if((rect.m_flags & FILLED.value) != 0)
			m_gl2.glBegin(GL.GL_POLYGON);
		else
			m_gl2.glBegin(GL.GL_LINE_LOOP);
		m_gl2.glVertex2d(rect.m_xmin, rect.m_ymin);
		m_gl2.glVertex2d(rect.m_xmax, rect.m_ymin);
		m_gl2.glVertex2d(rect.m_xmax, rect.m_ymax);
		m_gl2.glVertex2d(rect.m_xmin, rect.m_ymax);
		m_gl2.glEnd();
	}

	private void drawPolygon2D(Polygon2D poly)
	{
		m_gl2.glColor3d(poly.m_color.m_r/255., poly.m_color.m_g/255., poly.m_color.m_b/255.);
		if((poly.m_flags & FAT.value) != 0)
			m_gl2.glLineWidth(3.f);
		else
			m_gl2.glLineWidth(1.f);
		if((poly.m_flags & FILLED.value) != 0)
			m_gl2.glBegin(GL.GL_POLYGON);
		else
			m_gl2.glBegin(GL.GL_LINE_LOOP);
		for(int i = 0; i < poly.m_x.length; i++)
			m_gl2.glVertex2d(poly.m_x[i], poly.m_y[i]);
		m_gl2.glEnd();
	}

	private void drawPoint3D(Point3D point)
	{
		m_gl3.glColor3d(point.m_color.m_r/255., point.m_color.m_g/255., point.m_color.m_b/255.);
		if((point.m_flags & FAT.value) != 0)
			m_gl3.glPointSize(3.f);
		else
			m_gl3.glPointSize(1.f);
		m_gl3.glBegin(GL.GL_POINTS);
		m_gl3.glVertex3d(point.m_x, point.m_y, point.m_z);
		m_gl3.glEnd();
	}

	private void drawLine3D(Line3D line)
	{
		m_gl3.glColor3d(line.m_color.m_r/255., line.m_color.m_g/255., line.m_color.m_b/255.);
		if((line.m_flags & FAT.value) != 0)
			m_gl3.glLineWidth(3.f);
		else
			m_gl3.glLineWidth(1.f);
		m_gl3.glBegin(GL.GL_LINES);
		m_gl3.glVertex3d(line.m_x1, line.m_y1, line.m_z1);
		m_gl3.glVertex3d(line.m_x2, line.m_y2, line.m_z2);
		m_gl3.glEnd();
	}

  /* (non-Javadoc)
	 * @see balt.core.processes.FrameworkProcess#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config)
	{

	}

	/* (non-Javadoc)
	 * @see balt.core.processes.FrameworkProcess#run()
	 */
	@Override
	public void run()
	{
		setupWidgets();
		m_shell.open();
		while (!m_shell.isDisposed())
		{
			// sleep until either an explicit draw request or an event wakes us up
			m_display.sleep();
			handleExplicitRedraw();
			handleEvents();
		}
		m_display.dispose();
	}
}
