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
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.*;
import org.eclipse.swt.graphics.*;
import org.eclipse.swt.widgets.*;
import edu.ksu.cis.bnj.ver3.core.CPF;
public class BasicKDD_Pane extends Canvas
{
	CPF		_currentCPF;
	Color	back			= null;
	Color	textcolor		= null;
	Image	backbuffer		= null;
	Display	backbuffer_disp	= null;
	GC		back_gc			= null;
	int		x				= 0;
	public void event_widgetDisposed(DisposeEvent e)
	{
		back.dispose();
		textcolor.dispose();
		if (backbuffer != null) backbuffer.dispose();
		if (back_gc != null) back_gc.dispose();
	}
	public void event_paintControl(PaintEvent e)
	{
		//** ---SYSTEM---
		//** Double Buffer management
		//** -JMB!
		if (backbuffer == null)
		{
			backbuffer_disp = getDisplay();
			backbuffer = new Image(backbuffer_disp, e.width, e.height);
			back_gc = new GC(backbuffer);
		}
		if (e.width != backbuffer.getBounds().width || e.height != backbuffer.getBounds().height
				|| backbuffer_disp != getDisplay())
		{
			if (backbuffer_disp != getDisplay())
			{
				System.out.println("wtf?");
			}
			backbuffer.dispose();
			back_gc.dispose();
			backbuffer_disp = getDisplay();
			if (e.width > 0 && e.height > 0)
			{
				backbuffer = new Image(backbuffer_disp, e.width, e.height);
				back_gc = new GC(backbuffer);
			}
			else
			{
				backbuffer = null;
				back_gc = null;
			}
		}
		if (e.width <= 0 || e.height <= 0) return;
		// begin rendering on the back plane
		// paint the background color
		back_gc.setForeground(back);
		back_gc.fillRectangle(0, 0, e.width, e.height);
		// drawing onto empty surface
		back_gc.setForeground(textcolor);
		back_gc.drawLine(x, x, x + 100, x + 100);
		x++;
		if (x % 50 == 0) x = 0;
		// render to primary surface
		e.gc.drawImage(backbuffer, 0, 0);
	}
	public void event_mouseDown(MouseEvent e)
	{}
	public void event_mouseUp(MouseEvent e)
	{}
	public void event_mouseDoubleClick(MouseEvent e)
	{}
	public void event_mouseMove(MouseEvent e)
	{}
	public void event_focusGained(FocusEvent e)
	{}
	public void event_focusLost(FocusEvent e)
	{}
	public void event_controlResized(ControlEvent e)
	{}
	public void event_keyPressed(KeyEvent e)
	{}
	public void event_keyReleased(KeyEvent e)
	{}
	public void event_mouseEnter(MouseEvent e)
	{}
	public void event_mouseExit(MouseEvent e)
	{}
	public void event_mouseHover(MouseEvent e)
	{}
	public BasicKDD_Pane(Composite parent, int style)
	{
		//NO_BACKGROUND, NO_REDRAW_RESIZE and NO_MERGE_PAINTS
		//SWT.NO_BACKGROUND
		super(parent, style | SWT.NO_BACKGROUND);
		back = new Color(getDisplay(), 255, 255, 255);
		textcolor = new Color(getDisplay(), 0, 0, 55);
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
	}
}