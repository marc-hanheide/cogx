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
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.PaintEvent;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.graphics.Rectangle;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Slider;
import edu.ksu.cis.bnj.gui.tools.Constants;
import edu.ksu.cis.bnj.gui.tools.FocusRoute;
import edu.ksu.cis.bnj.gui.tools.RenderContext;
import edu.ksu.cis.bnj.gui.tools.Transformation;
public class KDDCanvas extends Canvas 
{
	protected Image				backbuffer				= null;
	protected Display			backbuffer_disp			= null;
	protected GC				back_gc					= null;
	protected boolean			_TranslationOn			= false;
	protected boolean			_LeftMouseDown			= false;
	protected boolean			_RightMouseDown			= false;
	protected Transformation	_Transform;
	protected Constants			_Constants;
	protected int				_SkipFrameCounter		= 0;
	protected int				_XCapture;
	protected int				_YCapture;
	protected int				_MouseEventFire			= 0;
	protected Point				_Cursor;
	protected RenderContext		_RenderingContext;
	protected boolean			_OnRenderLoopZoom2Fit	= false;
	protected int				_KeyScalingMode;
	protected int				_KeyPanningX			= 0;
	protected int				_KeyPanningY			= 0;
	protected int				_Click2DragMonitor;
	protected boolean			_DragMode				= false;
	protected Point				_TranslationTally;
	protected boolean			_HasKeyboardSignal		= false;

	private Slider _HorzSlider = null;
	private Slider _VertSlider = null;
	private int _HorzPosition;
	private int _VertPosition;
	private boolean _SkipSync;
	private FocusRoute _FocusRoute = null;
	
	boolean outFocus;
	
	public int drawMsg = 0;
	
	public void drawFinished()
	{
		drawMsg--;
		if(drawMsg < 0) drawMsg = 0;
	}
	
	protected void ourRedraw()
	{
		drawMsg++;
		if(drawMsg > 0)
		{
			redraw();
		}
	}
	
	public void setFocusRoute(FocusRoute FR)
	{
		_FocusRoute = FR;
	}
	
	public void mapGainFocus()
	{
		outFocus = false;
	}
	
	public void outFocusBeep()
	{
		ourRedraw();
		if(outFocus)
		{
		getDisplay().timerExec(100, new Runnable()
				{
					public void run()
					{
						outFocusBeep();
					}
				});
		}
	}
	
	public void mapLostFocus()
	{
		if(_FocusRoute != null)
			_FocusRoute.redoFocus();
		outFocus = true;
		getDisplay().timerExec(100, new Runnable()
				{
					public void run()
					{
						outFocusBeep();
					}
				});		
	}
	
	public void SyncSliders()
	{
		if(_Transform == null) return;
		if(_SkipSync)
		{
			_SkipSync = false;
			return;
		}
		else
		{
			_Transform.commit();
		}
		if(_HorzSlider != null)
			SyncH(_HorzSlider);
		if(_VertSlider != null)
			SyncV(_VertSlider);
	}
	
	public void SyncH(Slider H)
	{
		_HorzSlider = H;
		if(_Transform == null) return;
		int gMin = (_RenderingContext.Bounds.x < 0) ? _RenderingContext.Bounds.x : 0;
		int gMax = (_RenderingContext.Bounds.x + _RenderingContext.Bounds.width >  _Transform.w()) ? _RenderingContext.Bounds.x + _RenderingContext.Bounds.width  :  _Transform.w();
		
		int gDist =  gMax - gMin;
		H.setMinimum(0);
		H.setMaximum(gDist);
		H.setSelection( -gMin);
		H.setThumb(_Transform.w());
		_HorzPosition = -gMin;
		
		if(gDist <= _Transform.w())
		{
			H.setEnabled(false);
		}
		else
		{
			H.setEnabled(true);
		}
	}
	
	public void SyncV(Slider V)
	{
		_VertSlider = V;
		if(_Transform == null) return;
		int gMin = (_RenderingContext.Bounds.y < 0) ? _RenderingContext.Bounds.y : 0;
		int gMax = (_RenderingContext.Bounds.y + _RenderingContext.Bounds.height >  _Transform.h()) ? _RenderingContext.Bounds.y + _RenderingContext.Bounds.height  :  _Transform.h();
		int gDist =  gMax - gMin;
		V.setMinimum(0);
		V.setMaximum(gDist);
		V.setSelection( -gMin);
		V.setThumb(_Transform.h());
		_VertPosition = -gMin;
		if(gDist <= _Transform.h())
		{
			V.setEnabled(false);
		}
		else
		{
			V.setEnabled(true);
		}

	}
	public void RegisterSlide()
	{
		if(_Transform == null) return;

		int dX = 0;
		int dY = 0;
		if(_HorzSlider != null)
		{
			_SkipSync = true;
			dX = -(_HorzSlider.getSelection() - _HorzPosition);
			ourRedraw();
		}
		if(_VertSlider != null)
		{
			_SkipSync = true;
			dY = -(_VertSlider.getSelection() - _VertPosition);
			ourRedraw();
		}
		_Transform.settemp(dX, dY);
	}
	
	public void transformRedraw()
	{
		_SkipFrameCounter--;
		if (_SkipFrameCounter <= 0)
		{
			ourRedraw();
			_SkipFrameCounter = _Constants.TranslationFrameSkip;
		}
	}
	boolean _MessageThrown = false;
	boolean _KillSignal		= false;
	public void keyboardMovement()
	{
		boolean redo = false;
		if (_KeyScalingMode == 1)
		{
			_Transform.scale(_Constants.ScalingFactor);
			redo = true;
		}
		else if (_KeyScalingMode == -1)
		{
			_Transform.scale(1.0 / _Constants.ScalingFactor);
			redo = true;
		}
		if (_KeyPanningX != 0 || _KeyPanningY != 0)
		{
			_Transform.translate(_KeyPanningX, _KeyPanningY);
			redo = true;
		}
		if(_KillSignal)
		{
			redo = false;
			_KillSignal = false;
		}
		if (redo)
		{
			/*
			_MessageThrown = true;
			getDisplay().timerExec(8, new Runnable()
			{
				public void run()
				{
					keyboardMovement();
					our();
				}
			});
			*/
			// make the timer isn't faster than the drawing
		}
		else
		{
			//_MessageThrown = false;
		}
		
		_HasKeyboardSignal = redo;
	}
	void killMessage()
	{
		_KillSignal = true;
	}

	protected void onHeartBeat(int dT)
	{
		
	}
	
	public void HeartBeat()
	{
		
		if(_HasKeyboardSignal)
		{
			keyboardMovement();
			ourRedraw();
		}
		
		onHeartBeat(10);
		
		getDisplay().timerExec(10, new Runnable()
		{
			public void run()
			{
				if(!_KillSignal) HeartBeat();
			}
		});
	}
	
	public void throwKeyboardMovement()
	{
		_HasKeyboardSignal = true;
/*		if(!_MessageThrown)
		getDisplay().timerExec(8, new Runnable()
		{
			public void run()
			{
				keyboardMovement();
			}
		});
		*/
	}
	
	void SendStart()
	{
		_KillSignal = false;
		HeartBeat();
	}
	public void ReloadConstants()
	{
		_Constants = new Constants();
	}
	public boolean backbuffer(PaintEvent e)
	{
		if (e.width <= 0 || e.height <= 0) return false;
		if (backbuffer == null)
		{
			backbuffer_disp = getDisplay();
			if(backbuffer_disp == null) return false;
			backbuffer = new Image(backbuffer_disp, e.width, e.height);
			back_gc = new GC(backbuffer);
		}
		if (e.width != backbuffer.getBounds().width || e.height != backbuffer.getBounds().height
				|| backbuffer_disp != getDisplay())
		{
			if (backbuffer_disp != getDisplay())
			{
				_Constants.clean();
				_Constants.init(getDisplay());
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

		return true;
	}
	public void drawBack(PaintEvent e)
	{
		_RenderingContext.gc.setBackground(_Constants.Colors._DefaultBackground);
		_RenderingContext.gc.setForeground(_Constants.Colors._DefaultText);
		_RenderingContext.gc.fillRectangle(0, 0, e.width, e.height);
		_RenderingContext.gc.setForeground(_Constants.Colors._GridColor);
		_RenderingContext.gc.setLineStyle(_Constants.GridLineStyle);
		_RenderingContext.gc.setLineWidth(_Constants.GridLineWidth);
		int gX = _Transform.itx(0);
		int gY = _Transform.ity(0);
		gX /= 50;
		gX *= 50;
		gY /= 50;
		gY *= 50;
		gX = _Transform.tx(gX);
		gY = _Transform.ty(gY);
		int dZ = (int) _Transform.s(_Constants.GridDistance);
		for (int z = -dZ; z <= e.width + dZ; z += dZ)
		{
			_RenderingContext.gc.drawLine(gX + z, 0, gX + z, e.height);
		}
		for (int z = -dZ; z <= e.height + dZ; z += dZ)
		{
			_RenderingContext.gc.drawLine(0, gY + z, e.width, gY + z);
		}
		_RenderingContext.gc.setBackground(_Constants.Colors._DefaultBackground);
		_RenderingContext.gc.setForeground(_Constants.Colors._DefaultText);
	}
	public void setupRenderingContext()
	{
		_RenderingContext.gc = back_gc;
		_RenderingContext.constants = _Constants;
		_RenderingContext.Trans = _Transform;
	}
	public void translate_mousedown(MouseEvent e)
	{
		_XCapture = e.x;
		_YCapture = e.y;
		if (_Cursor != null)
		{
			if (_Cursor.x != _XCapture || _Cursor.y != _YCapture)
			{
				_Cursor.x = _XCapture;
				_Cursor.y = _YCapture;
				ourRedraw();
			}
		}
		if (e.button == _Constants.LeftMouse) _LeftMouseDown = true;
		if (e.button == _Constants.RightMouse) _RightMouseDown = true;
		_MouseEventFire++;
		if ((((e.stateMask & SWT.CTRL) != 0 && _LeftMouseDown) || (_LeftMouseDown && _RightMouseDown)))
		{
			_TranslationOn = true;
			_MouseEventFire = 0;
		}
		endDragMode();
	}
	public void translate_mouseup(MouseEvent e)
	{
		_MouseEventFire = 0;
		if (e.button == _Constants.LeftMouse) _LeftMouseDown = false;
		if (e.button == _Constants.RightMouse) _RightMouseDown = false;
		if (!_LeftMouseDown)
		{
			_TranslationOn = false;
			ourRedraw();
		}
		if (!_RightMouseDown)
		{
			_TranslationOn = false;
			ourRedraw();
		}
	}
	public void mapCenter()
	{
		if (_RenderingContext.Bounds != null)
		{
			_Transform.Center(_RenderingContext.Bounds);
			ourRedraw();
		}
	}
	public void mapZoom2Fit()
	{
		if (_RenderingContext.Bounds != null)
		{
			if (_RenderingContext.Bounds.width > 10 && _RenderingContext.Bounds.height > 10)
			{
				_OnRenderLoopZoom2Fit = true;
				ourRedraw();
			}
		}
	}
	public void mapZoomIn()
	{
		_Transform.scale(1.10);
		ourRedraw();
	}
	public void mapZoomOut()
	{
		_Transform.scale(0.90909090909090909090909090909091);
		ourRedraw();
	}
	public void mapZoom100()
	{
		_Transform.normalizescale();
		ourRedraw();
	}
	public void keyMovement(KeyEvent e)
	{
		if (e.character == 'q')
		{
			_KeyScalingMode = 1;
			throwKeyboardMovement();
			ourRedraw();
		}
		else if (e.character == 'a')
		{
			_KeyScalingMode = -1;
			throwKeyboardMovement();
			ourRedraw();
		}
		else if (e.character == 'z')
		{
			mapZoom100();
			ourRedraw();
		}
		else if (e.character == 'c')
		{
			mapCenter();
			ourRedraw();
		}
		else if (e.character == 'x')
		{
			mapZoom2Fit();
			ourRedraw();
		}
		else
		{
			if (e.keyCode == SWT.ARROW_LEFT)
			{
				_KeyPanningX = _Constants.TranslationDelta;
				throwKeyboardMovement();
				ourRedraw();
			}
			else if (e.keyCode == SWT.ARROW_RIGHT)
			{
				_KeyPanningX = -_Constants.TranslationDelta;
				throwKeyboardMovement();
				ourRedraw();
			}
			if (e.keyCode == SWT.ARROW_UP)
			{
				_KeyPanningY = _Constants.TranslationDelta;
				throwKeyboardMovement();
				ourRedraw();
			}
			else if (e.keyCode == SWT.ARROW_DOWN)
			{
				_KeyPanningY = -_Constants.TranslationDelta;
				throwKeyboardMovement();
				ourRedraw();
			}
		}
	}
	public void invertKeyMovement(KeyEvent e)
	{
		if (e.character == 'q')
		{
			_KeyScalingMode = 0;
			ourRedraw();
		}
		else if (e.character == 'a')
		{
			_KeyScalingMode = 0;
			ourRedraw();
		}
		else if (e.keyCode == SWT.ARROW_LEFT)
		{
			_KeyPanningX = 0;
			ourRedraw();
		}
		else if (e.keyCode == SWT.ARROW_RIGHT)
		{
			_KeyPanningX = 0;
			ourRedraw();
		}
		if (e.keyCode == SWT.ARROW_UP)
		{
			_KeyPanningY = 0;
			ourRedraw();
		}
		else if (e.keyCode == SWT.ARROW_DOWN)
		{
			_KeyPanningY = 0;
			ourRedraw();
		}
	}
	public KDDCanvas(Composite parent, int style)
	{
		super(parent, style | SWT.NO_BACKGROUND);
		_Transform = new Transformation();
		_Constants = new Constants();
		_Constants.init(getDisplay());
		_RenderingContext = new RenderContext();
		_RenderingContext.Bounds = new Rectangle(0, 0, 1, 1);
		_Cursor = new Point(0, 0);
		_Click2DragMonitor = _Constants.DragDifferential;
		_TranslationTally = new Point(0, 0);
	}
	public void endDragMode()
	{
		if (_DragMode)
		{
			_Click2DragMonitor = _Constants.DragDifferential;
			_DragMode = false;
			ourRedraw();
		}
	}
	public boolean enterDragmode()
	{
		if (_LeftMouseDown && !_TranslationOn && !_RightMouseDown && !_DragMode) _Click2DragMonitor--;
		if (_Click2DragMonitor <= 0 && !_DragMode)
		{
			_DragMode = true;
			return true;
		}
		return false;
	}
	public void DragModeRefresh()
	{
		if (_DragMode)
		{
			if (_SkipFrameCounter <= 0)
			{
				ourRedraw();
				_SkipFrameCounter = _Constants.DragFrameSkip;
			}
			_SkipFrameCounter--;
		}
	}
}