/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.bnj.gui;
import java.util.ArrayList;
import java.util.Iterator;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.*;
import org.eclipse.swt.graphics.*;
import org.eclipse.swt.widgets.*;
import edu.ksu.cis.bnj.gui.rendering.RenderEdge;
import edu.ksu.cis.bnj.gui.rendering.RenderNetwork;
import edu.ksu.cis.bnj.gui.rendering.RenderNode;
import edu.ksu.cis.bnj.gui.tools.LanguageUnicodeParser;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.dynamic.DynamicTyping;
import edu.ksu.cis.bnj.ver3.inference.exact.LS;
import edu.ksu.cis.util.GlobalOptions;
import edu.ksu.cis.util.graph.core.Edge;
import edu.ksu.cis.util.graph.core.Graph;
import edu.ksu.cis.util.graph.core.Vertex;
import edu.ksu.cis.util.graph.core.Visitor;
import edu.ksu.cis.util.graph.visualization.Markup;
import edu.ksu.cis.util.graph.visualization.VisualizationController;
import edu.ksu.cis.util.graph.visualization.operators.ColorLegendMap;
public class NetworkVisualize extends KDDCanvas implements Visitor
{
	private RenderNetwork			REN			= null;
	private BeliefNetwork			_bn			= null;
	private VisualizationController	VC			= null;
	private Graph					_frame		= null;
	private ArrayList				_markings	= null;
	private Menu					menuNetwork;
	public void event_widgetDisposed(DisposeEvent e)
	{
		_Constants.clean();
		if (backbuffer != null) backbuffer.dispose();
		if (back_gc != null) back_gc.dispose();
	}
	public void Restart()
	{
		if(!REN._bn.isInfluenceDiagram() && !DynamicTyping.IsDynamic(REN._bn,true))
		{
			_Transform = REN._trans;
			_bn = REN._bn;
			if (_bn == null) return;
			VC = new VisualizationController(_bn.getGraph());
			/*
			 BuildCliqueTree Moral = new BuildCliqueTree();
			 Moral.setGraph(_bn.getGraph());
			 Moral.setVisualization(VC);
			 Moral.execute();
			 */
			if (_bn.getGraph().getNumberOfVertices() > 1)
			{
				LS ls = new LS();
				ls.run(_bn, VC);
				_frame = VC.getFrame();
				_markings = VC.getMarkings();
				codepage = VC.getCode();
				_active = VC.getActiveLine();
				header = VC.getHeader();
			}
		}
		else
		{
			header = "Unable to visualize Influence Diagrams/Dynamic BN/Empty BN";
		}
		ourRedraw();
	}
	public void SetNetwork(RenderNetwork ren)
	{
		REN = ren;
		_Transform = ren._trans;
	}
	int			_active;
	String[]	codepage;
	String		header;
	void drawCodepage(PaintEvent e)
	{
		if (codepage == null) return;
		int offY = e.height;
		for (int i = codepage.length - 1; i >= 0; i--)
		{
			if (i != _active)
			{
				_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._VisCodePageBackInactive);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisCodePageForeInactive);
				_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.CodePageInactive);
				offY -= (_RenderingContext.gc.textExtent(codepage[i]).y + 3);
				_RenderingContext.gc.drawText(codepage[i], 4, offY);
			}
			else
			{
				_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._VisCodePageBackActive);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisCodePageBackActive);
				_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.CodePageActive);
				Point dim = _RenderingContext.gc.textExtent(codepage[i]);
				offY -= (dim.y + 3);
				_RenderingContext.gc.fillRectangle(0, offY - 1, dim.x + 8, dim.y + 1);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisCodePageForeActive);
				_RenderingContext.gc.drawText(codepage[i], 4, offY);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisCodePageForeBoxActive);
				_RenderingContext.gc.drawRectangle(0, offY - 1, dim.x + 8, dim.y + 1);
			}
		}
	}
	Point[]	_size_buffer;
	public boolean onVertex(Vertex V)
	{
		int nX = (int) _Transform.tx(V.getx());
		int nY = (int) _Transform.ty(V.gety());
		Object o = V.getObject();
		if (o instanceof BeliefNode)
		{
			//System.out.println("V.color:" + V.getAttrib(2));
			BeliefNode bnode = (BeliefNode) o;
			_size_buffer[V.loc()] = RenderNode.rBefiefNode(_RenderingContext.gc, nX, nY, bnode, V, _Constants, false);
			if (V.getAttrib(3) > 0)
			{
				Point s = _size_buffer[V.loc()];
				int px = (int) (_Transform.tx(V.getx()) + s.x / 2
						* _RenderingContext.constants.VisNodeOrderPlacement_Xf);
				int py = (int) (_Transform.ty(V.gety()) + s.y / 2
						* _RenderingContext.constants.VisNodeOrderPlacement_Yf);
				_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.NodeOrder);
				char alphachar = 0x03B1;
				String X = alphachar + "=" + V.getAttrib(3);
				Point z = _RenderingContext.gc.textExtent(X);
				_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._VisOrderBox);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisOrderBoxBorder);
				_RenderingContext.gc.fillRoundRectangle(px - 2, py - 2, z.x + 4, z.y + 4, 5, 5);
				_RenderingContext.gc.drawRoundRectangle(px - 2, py - 2, z.x + 4, z.y + 4, 5, 5);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisOrderBoxFore);
				//RC.gc.textExtent( )
				_RenderingContext.gc.drawText(X, px, py);
			}
		}
		/*
		 if( o instanceof HashSet)
		 {
		 BeliefNode bnode = new BeliefNode(V.getName(), new Discrete());
		 bnode.setOwner(V);
		 _size_buffer[V.loc()] = RenderNode.rBefiefNode(_RenderingContext.gc, nX, nY, bnode, V, _Constants, false);
		 }
		 */
		else
		//if( o instanceof Clique)
		{
			BeliefNode bnode = new BeliefNode(V.getName(), new Discrete());
			bnode.setOwner(V);
			_size_buffer[V.loc()] = RenderNode.rBefiefNode(_RenderingContext.gc, nX, nY, bnode, V, _Constants, false);
		}
		int mX = nX - _size_buffer[V.loc()].x / 2;
		int mY = nY - _size_buffer[V.loc()].y / 2;
		if (mX < _RenderingContext.Bounds.x) _RenderingContext.Bounds.x = mX;
		if (mY < _RenderingContext.Bounds.y) _RenderingContext.Bounds.y = mY;
		mX = nX + _size_buffer[V.loc()].x / 2;
		mY = nY + _size_buffer[V.loc()].y / 2;
		if (mX > _RenderingContext.Bounds.width) _RenderingContext.Bounds.width = mX;
		if (mY > _RenderingContext.Bounds.height) _RenderingContext.Bounds.height = mY;
		return true;
	}
	public boolean onEdge(Edge E)
	{
		_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._Edge_Color);
		_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._Edge_Color);
		_RenderingContext.gc.setLineWidth(_RenderingContext.constants.EdgeWidth);
		_RenderingContext.gc.setLineStyle(_RenderingContext.constants.EdgeStyle);
		if (_frame.getConnectedness(E.s(), E.d()) == 2)
		{
			RenderEdge.rRenderEdge2bi((int) _Transform.tx(E.s().getx()), (int) _Transform.ty(E.s().gety()),
					(int) _Transform.tx(E.d().getx()), (int) _Transform.ty(E.d().gety()), _RenderingContext,
					_size_buffer[E.d().loc()]);
		}
		else
		{
			RenderEdge.rRenderEdge2((int) _Transform.tx(E.s().getx()), (int) _Transform.ty(E.s().gety()),
					(int) _Transform.tx(E.d().getx()), (int) _Transform.ty(E.d().gety()), _RenderingContext,
					_size_buffer[E.d().loc()]);
		}
		return true;
	}
	public boolean onTempEdge(Edge E)
	{
		_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._EdgeTemp_Color);
		_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._EdgeTemp_Color);
		_RenderingContext.gc.setLineWidth(_RenderingContext.constants.EdgeTempWidth);
		_RenderingContext.gc.setLineStyle(_RenderingContext.constants.EdgeTempStyle);
		if (!E.isDirected())
		{
			RenderEdge.rRenderEdge2bi((int) _Transform.tx(E.s().getx()), (int) _Transform.ty(E.s().gety()),
					(int) _Transform.tx(E.d().getx()), (int) _Transform.ty(E.d().gety()), _RenderingContext,
					_size_buffer[E.d().loc()]);
		}
		else
		{
			RenderEdge.rRenderEdge2((int) _Transform.tx(E.s().getx()), (int) _Transform.ty(E.s().gety()),
					(int) _Transform.tx(E.d().getx()), (int) _Transform.ty(E.d().gety()), _RenderingContext,
					_size_buffer[E.d().loc()]);
		}
		return true;
	}
	public void draw()
	{
		_RenderingContext.Bounds.x = 100000;
		_RenderingContext.Bounds.y = 100000;
		_RenderingContext.Bounds.width = -100000;
		_RenderingContext.Bounds.height = -100000;
		_frame.apply(this, true, true);
		try
		{
			ArrayList tempEdges = VC.getTempEdges();
			for (Iterator it = tempEdges.iterator(); it.hasNext();)
			{
				Edge TEC = (Edge) it.next();
				onTempEdge(TEC);
			}
		}
		catch (Exception e)
		{}
		_frame.apply(this, true, false);
		_RenderingContext.Bounds.width -= _RenderingContext.Bounds.x;
		_RenderingContext.Bounds.height -= _RenderingContext.Bounds.y;
		try
		{
			ArrayList legend = VC.getColorLegend();
			_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.ColorLegend);
			int h = 4;
			for (Iterator it = legend.iterator(); it.hasNext();)
			{
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._ColorLegendBorder);
				_RenderingContext.gc.setLineStyle(SWT.LINE_SOLID);
				_RenderingContext.gc.setLineWidth(2);
				ColorLegendMap CLM = (ColorLegendMap) it.next();
				Point d = _RenderingContext.gc.textExtent(CLM.Name);
				_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._NodeBackColor[CLM.ColorIdx]);
				_RenderingContext.gc.fillRoundRectangle(_RenderingContext.Trans.w() - 28, h, 24, 24, 16, 16);
				_RenderingContext.gc.drawRoundRectangle(_RenderingContext.Trans.w() - 28, h, 24, 24, 16, 16);
				//				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._NodeColor[CLM.ColorIdx]);
				_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._DefaultBackground);
				_RenderingContext.gc
						.drawText(CLM.Name, _RenderingContext.Trans.w() - 28 - d.x - 10, h + (12 - d.y / 2));
				h += 26;
			}
		}
		catch (Exception e)
		{}
	}
	public void event_paintControl(PaintEvent e)
	{
		_Transform = REN._trans;
		_Transform.setWindow(e.width, e.height);
		//** ---SYSTEM---
		//** Double Buffer management
		//** -JMB!
		if (!super.backbuffer(e)) return;
		// begin rendering on the back plane
		// paint the background color
		// drawing onto empty surface
		setupRenderingContext();
		super.drawBack(e);
		if (header != null)
		{
			_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisHeader);
			_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.VisHeader);
			_RenderingContext.gc.drawText(header, 0, 0);
		}
		drawCodepage(e);
		if (_frame != null)
		{
			if (_frame.getVertices().length > _size_buffer.length)
			{
				_size_buffer = new Point[_frame.getVertices().length];
			}
			if (_markings != null)
			{
				_RenderingContext.gc.setBackground(_RenderingContext.constants.Colors._DefaultBackground);
				_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisMarkingColor);
				_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.VisMarking);
				for (Iterator it = _markings.iterator(); it.hasNext();)
				{
					Markup M = (Markup) it.next();
					_RenderingContext.gc.drawText(M.msg, (int) (_RenderingContext.Trans.tx(M.x)),
							(int) (_RenderingContext.Trans.ty(M.y)));
				}
			}
			draw();
			if (_OnRenderLoopZoom2Fit && _frame.getNumberOfVertices() > 0)
			{

				_Transform.normalizescale();
				draw();
				_Transform.Zoom2Fit(_RenderingContext.Bounds);
				draw();
				_Transform.Center(_RenderingContext.Bounds);
				super.drawBack(e);
				if (header != null)
				{
					_RenderingContext.gc.setForeground(_RenderingContext.constants.Colors._VisHeader);
					_RenderingContext.gc.setFont(_RenderingContext.constants.Fonts.VisHeader);
					_RenderingContext.gc.drawText(header, 0, 0);
				}
				drawCodepage(e);
				draw();
				//		ourRedraw();
			}
			_OnRenderLoopZoom2Fit = false;			
			SyncSliders();
		}
		if (_Constants.RenderNetworkBoundingBox)
		{
			_RenderingContext.gc.drawRectangle(_RenderingContext.Bounds);
		}
		// render to primary surface
		e.gc.drawImage(backbuffer, 0, 0);
		SyncSliders();
		drawFinished();
	}
	public void fireRightClick(int x, int y)
	{
		menuNetwork.setVisible(true);
	}
	public void event_mouseDown(MouseEvent e)
	{
		translate_mousedown(e);
		if (_DragMode)
		{
			_Click2DragMonitor = _Constants.DragDifferential;
			_DragMode = false;
			ourRedraw();
		}
	}
	public void event_mouseUp(MouseEvent e)
	{
		if (!_LeftMouseDown && _RightMouseDown && _MouseEventFire == 1) fireRightClick(_Cursor.x, _Cursor.y);
		translate_mouseup(e);
		if (_DragMode)
		{
			ourRedraw();
		}
		_Click2DragMonitor = _Constants.DragDifferential;
		_DragMode = false;
	}
	public void event_mouseDoubleClick(MouseEvent e)
	{}
	public void event_mouseMove(MouseEvent e)
	{
		if (_DragMode)
		{
			int dX = e.x - _XCapture;
			int dY = e.y - _YCapture;
			_XCapture = e.x;
			_YCapture = e.y;
			//REN.translate(dX, dY, _Transform);
			ourRedraw();
		}
		if (_TranslationOn)
		{
			int dX = e.x - _XCapture;
			int dY = e.y - _YCapture;
			_XCapture = e.x;
			_YCapture = e.y;
			_Transform.translate(dX, dY);
			transformRedraw();
		}
	}
	public void event_focusGained(FocusEvent e)
	{
		SendStart();
		mapGainFocus();
	}
	public void event_focusLost(FocusEvent e)
	{
		if(!recording)
		{
			Pause();
			killMessage();
			mapLostFocus();
		}
	}
	public void event_controlResized(ControlEvent e)
	{}
	public void event_keyPressed(KeyEvent e)
	{
		if (e.character == 'p')
			Forward();
		else if (e.character == 'b')
			Back();
		else if (e.character == 'r')
			Record();
		else if (e.character == 's')
			Stop();
		else if (e.character == 'u')
			Pause();
		else if (e.character == 'm')
			SkipForward();
		else if (e.character == 'n')
			SkipBack();
		else
		{
			keyMovement(e);
		}
	}
	public void event_keyReleased(KeyEvent e)
	{
		invertKeyMovement(e);
	}
	public void event_mouseEnter(MouseEvent e)
	{}
	public void event_mouseExit(MouseEvent e)
	{}
	public void event_mouseHover(MouseEvent e)
	{}
	int	spd			= 1;
	int	velo		= 1;	// 10 fps
	int	frame		= 150;
	int	req			= 0;
	int	framestep	= 0;
	double	timeCount	= 0;
	boolean recording = false;
	public void Record()
	{
		Restart();
		recording = true;
		speedFactor = 100;
		framestep = 0;
	}
	
	public void Step()
	{
		if(VC == null) return;
		if (velo != 0)
		{
			if (recording)
			{
				ImageLoader IL = new ImageLoader();
				IL.data = new ImageData[1];
				IL.data[0] = backbuffer.getImageData();
				String filename = "0000000000" + framestep;
				filename = "vis_" + filename.substring(filename.length() - 9) + ".jpeg";
				IL.save(filename, SWT.IMAGE_JPEG);
				framestep++;
			}
			if (VC != null && velo > 0)
			{
				VC.applyNextFrame();
				if(VC.doZoom2Fit())
				{
					_OnRenderLoopZoom2Fit = true;
				}
				timeCount += VC.getTime2Next();
				_frame = VC.getFrame();
				_markings = VC.getMarkings();
				codepage = VC.getCode();
				_active = VC.getActiveLine();
				header = VC.getHeader();
				ourRedraw();
			}
			else if (VC != null && velo < 0)
			{
				VC.applyPreviousFrame();
				timeCount += VC.getTime2Prev();
				_frame = VC.getFrame();
				_markings = VC.getMarkings();
				codepage = VC.getCode();
				_active = VC.getActiveLine();
				header = VC.getHeader();
				ourRedraw();
			}
			/*
			 if(!VC.isStopped())
			 {
			 if(factor > 0 && cntDown != 0)
			 {
			 getDisplay().timerExec ((rSpeed + 1) * factor, new Runnable () {
			 public void run () {
			 Step();
			 }
			 });
			 }
			 if(cntDown==0)
			 {
			 getDisplay().timerExec (1, new Runnable () {
			 public void run () {
			 Step();
			 }
			 });
			 }
			 }
			 */
		}
	}
	double	speedFactor	= 1.0;
	public void setSpeed(int idx)
	{
		double[] speed = new double[] { -8, -4, -2, -1, -0.5, -.25, 0, .25, .5, 1, 2, 4, 8 };
		if (idx == 6)
		{
			speedFactor = 0;
		}
		else
		{
			speedFactor = Math.abs(speed[idx]);
			if (speed[idx] < 0)
				velo = -1;
			else
				velo = 1;
		}
	}

	protected void onHeartBeat(int dT)
	{
		double timeCorrect = GlobalOptions.getInstance().getDouble("time_mult", .5);
		double tcdt = (dT * timeCorrect * speedFactor);
		timeCount -= tcdt;
		if (timeCount <= 0)
		{
			Step();
		}
	}
	public void SkipBack()
	{
		if(VC != null)
		{
			VC.SkipBackwardUntilPrevHeader();
			_frame = VC.getFrame();
			_markings = VC.getMarkings();
			codepage = VC.getCode();
			_active = VC.getActiveLine();
			header = VC.getHeader();
			ourRedraw();
		}
	}
	public void SkipForward()
	{
		if(VC != null)
		{
			VC.SkipForwardUntilNextHeader();
			_frame = VC.getFrame();
			_markings = VC.getMarkings();
			codepage = VC.getCode();
			_active = VC.getActiveLine();
			header = VC.getHeader();
			ourRedraw();
		}
	}
	public void Pause()
	{
		velo = 0;
	}
	public void Stop()
	{
		velo = 0;
		timeCount = 0;
		Restart();
	}
	public void Forward()
	{
		velo = 1;
	}
	public void StepForward()
	{
		velo = 1;
		Step();
		velo = 0;
	}
	public void StepBack()
	{
		velo = -1;
		Step();
		velo = 0;
	}
	public void Back()
	{
		velo = -1;
		getDisplay().timerExec(100, new Runnable()
		{
			public void run()
			{
				Step();
			}
		});
	}
	public NetworkVisualize(Composite parent, int style)
	{
		//NO_BACKGROUND, NO_REDRAW_RESIZE and NO_MERGE_PAINTS
		//SWT.NO_BACKGROUND
		super(parent, style | SWT.NO_BACKGROUND);
		_size_buffer = new Point[10];
		addDisposeListener(new DisposeListener()
		{
			public void widgetDisposed(DisposeEvent e)
			{
				event_widgetDisposed(e);
			}
		});
		addPaintListener(new PaintListener()
		{
			public void paintControl(PaintEvent e)
			{
				event_paintControl(e);
			}
		});
		addMouseListener(new MouseAdapter()
		{
			public void mouseDown(MouseEvent e)
			{
				event_mouseDown(e);
			}
			public void mouseUp(MouseEvent e)
			{
				event_mouseUp(e);
			}
			public void mouseDoubleClick(MouseEvent e)
			{
				event_mouseDoubleClick(e);
			}
		});
		addMouseMoveListener(new MouseMoveListener()
		{
			public void mouseMove(MouseEvent e)
			{
				event_mouseMove(e);
			}
		});
		addFocusListener(new FocusAdapter()
		{
			public void focusGained(FocusEvent e)
			{
				event_focusGained(e);
			}
			public void focusLost(FocusEvent e)
			{
				event_focusLost(e);
			}
		});
		addControlListener(new ControlAdapter()
		{
			public void controlResized(ControlEvent e)
			{
				event_controlResized(e);
			}
		});
		addKeyListener(new KeyAdapter()
		{
			public void keyPressed(KeyEvent e)
			{
				event_keyPressed(e);
			}
			public void keyReleased(KeyEvent e)
			{
				event_keyReleased(e);
			}
		});
		addMouseTrackListener(new MouseTrackAdapter()
		{
			public void mouseEnter(MouseEvent e)
			{
				event_mouseEnter(e);
			}
			public void mouseExit(MouseEvent e)
			{
				event_mouseExit(e);
			}
			public void mouseHover(MouseEvent e)
			{
				event_mouseHover(e);
			}
		});
		createPopUps();
	}
	public void createPopUps()
	{
		LanguageUnicodeParser LUP = LanguageUnicodeParser.getInstance();
		menuNetwork = new Menu(this.getShell(), SWT.POP_UP);
		for (int i = 0; i < 13; i++)
		{
			final Integer I = new Integer(i);
			{
				MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
				item.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						setSpeed(I.intValue());
					}
				});
				item.setText(LUP.get("scroll" + i));
			}
		}
	}
}