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
import edu.ksu.cis.bnj.gui.dialogs.BeliefNodeEditorDlg;
import edu.ksu.cis.bnj.gui.rendering.RenderCPF;
import edu.ksu.cis.bnj.gui.rendering.RenderNetwork;
import edu.ksu.cis.bnj.gui.rendering.RenderNode;
import edu.ksu.cis.bnj.gui.tools.Constants;
import edu.ksu.cis.bnj.gui.tools.LanguageUnicodeParser;
import edu.ksu.cis.bnj.gui.tools.Layouts;
import edu.ksu.cis.bnj.gui.tools.RenderContext;
import edu.ksu.cis.bnj.gui.tools.SelectionContext;
import edu.ksu.cis.bnj.gui.tools.Transformation;
import edu.ksu.cis.bnj.gui.tools.undo.UndoCPFEdit;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
import edu.ksu.cis.bnj.ver3.inference.cutset.LoopCutset;
import edu.ksu.cis.util.graph.coloring.Clear;
import edu.ksu.cis.util.graph.coloring.InOut;
import edu.ksu.cis.util.graph.core.Edge;
import edu.ksu.cis.util.graph.core.Vertex;
import edu.ksu.cis.util.graph.layout.LayRandomPolyTree;
import edu.ksu.cis.util.graph.layout.LayPolyTree;
public class NetworkEdit extends KDDCanvas
{
	private RenderNetwork	REN;
	private boolean			edgeMode	= false;
	private boolean			edgeDo		= false;
	private Text			_spot		= null;
	private CPF				currentCPF	= null;
	private int				currentVal;
	private RenderCPF		resultrendering;
	private Menu			menuNode;
	private Menu			menuEdge;
	private Menu			menuNetwork;
	private NetworkEdit		me;
	private BeliefNode		_edit;
	private Edge			_edge;
	private boolean			_DoDoublePass;
	boolean isUtility = false;
	
	public void ReInit()
	{
		_DoDoublePass = true;
		REN = null;
		_Transform = null;
		_SkipFrameCounter = 0;
		_KeyScalingMode = 0;
		_TranslationOn = false;
		_LeftMouseDown = false;
		_RightMouseDown = false;
		_Click2DragMonitor = 0;
		_DragMode = false;
		edgeMode = false;
		edgeDo = false;
		_XCapture = 0;
		_YCapture = 0;
		_Cursor = null;
		_MouseEventFire = 0;
		_Constants = null;
		_spot = null;
		currentCPF = null;
		currentVal = -1;
		_OnRenderLoopZoom2Fit = false;
		_KeyPanningX = 0;
		_KeyPanningY = 0;
		REN = new RenderNetwork();
		_Transform = new Transformation();
		_Constants = new Constants();
		_Constants.init(getDisplay());
		_RenderingContext = new RenderContext();
		_RenderingContext.Bounds = new Rectangle(0, 0, 1, 1);
		_Cursor = new Point(0, 0);
		_Click2DragMonitor = _Constants.DragDifferential;
		setFocus();
	}
	
	public void Snap2Grid(int grid)
	{
		REN.snap2grid(grid);
		ourRedraw();
	}
	
	public void createPopUps()
	{
		me = this;
		LanguageUnicodeParser LUP = LanguageUnicodeParser.getInstance();
		menuNode = new Menu(this.getShell(), SWT.POP_UP);
		{
			MenuItem item = new MenuItem(menuNode, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					if (_edit != null)
					{
						BeliefNodeEditorDlg bne = new BeliefNodeEditorDlg();
						bne.open(getShell(), _Constants, _edit, REN._bn, REN._UndoContext);
					}
				}
			});
			item.setText(LUP.get("edit_node"));
		}
		{
			new MenuItem(menuNode, SWT.SEPARATOR);
		}
		{
			MenuItem item = new MenuItem(menuNode, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					if (_edit != null)
					{
						mapDeleteNode();
					}
				}
			});
			item.setText(LUP.get("remove"));
		}
		menuEdge = new Menu(this.getShell(), SWT.POP_UP);
		{
			MenuItem item = new MenuItem(menuEdge, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					if (_edge != null)
					{
						Vertex S = _edge.s();
						Vertex D = _edge.d();
						BeliefNode A = ((RenderNode) S.getObject()).node;
						BeliefNode B = ((RenderNode) D.getObject()).node;
						REN.Disconnect(S, D, A, B);
						REN.Connect(D, S, B, A);
						me.ourRedraw();
					}
				}
			});
			item.setText(LUP.get("invert"));
		}
		{
			new MenuItem(menuEdge, SWT.SEPARATOR);
		}
		{
			MenuItem item = new MenuItem(menuEdge, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					if (_edge != null)
					{
						Vertex S = _edge.s();
						Vertex D = _edge.d();
						BeliefNode A = ((RenderNode) S.getObject()).node;
						BeliefNode B = ((RenderNode) D.getObject()).node;
						REN.Disconnect(S, D, A, B);
						me.ourRedraw();
					}
				}
			});
			item.setText(LUP.get("remove"));
		}
		menuNetwork = new Menu(this.getShell(), SWT.POP_UP);
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					mapCenter();
				}
			});
			item.setText(LUP.get("tool_center"));
		}
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					mapZoom2Fit();
				}
			});
			item.setText(LUP.get("tool_zoom2fit"));
		}
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					mapZoomIn();
				}
			});
			item.setText(LUP.get("tool_zoomin"));
		}
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					mapZoomOut();
				}
			});
			item.setText(LUP.get("tool_zoomout"));
		}
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					mapZoom100();
				}
			});
			item.setText(LUP.get("tool_zoom100"));
		}
		{
			new MenuItem(menuNetwork, SWT.SEPARATOR);
		}		
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					Snap2Grid(50);
				}
			});
			item.setText(LUP.get("snap2grid"));
		}
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					SetAllCpf(true);
				}
			});
			item.setText(LUP.get("showallcpf"));
		}
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					SetAllCpf(false);
				}
			});
			item.setText(LUP.get("shownocpf"));
		}
		{
			new MenuItem(menuNetwork, SWT.SEPARATOR);
		}			
		{
			MenuItem item = new MenuItem(menuNetwork, SWT.PUSH);
			item.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					LayPolyTree.apply(REN._bn.getGraph());
					_OnRenderLoopZoom2Fit = true;
					ourRedraw();
				}
			});
			item.setText(LUP.get("polytreelayout"));
		}
	}
	public void killPopUps()
	{
		menuNode.dispose();
		menuEdge.dispose();
		menuNetwork.dispose();
	}
	public void init(BeliefNetwork bn)
	{
		//_network = bn;
		REN.construct(bn);
		if (bn.getNodes().length > 0) _OnRenderLoopZoom2Fit = true;
		ourRedraw();
		_RenderingContext.Bounds = new Rectangle(0, 0, 1, 1);
		setFocus();
	}
	public void Enter()
	{
		REN.Switch2Edit();
		setFocus();
	}
	public void undo()
	{
		_DoDoublePass = true;
		REN.Undo();
		ourRedraw();
		setFocus();
	}
	public Transformation getTransformation()
	{
		return _Transform;
	}

	public boolean verify(String s)
	{
		// we try the parser
		try
		{Double.parseDouble(s); return true;}
		catch (Exception e)
		{
			try
			{Double.parseDouble("0"+s+"0"); return true;}
			catch (Exception e2)
			{
				try
				{Double.parseDouble(s+".0"); return true;}
				catch (Exception e3)
				{
					return false;
				}
			}
		}
	}
	
	private void beginCPFedit(CPF cpf, int v)
	{
		if (cpf == null) return;
		if (v >= cpf.size()) return;
		currentCPF = cpf;
		currentVal = v;
		_spot.setEnabled(true);
		_spot.setFont(_Constants.Fonts.CPF_Body);
		_spot.setVisible(true);
		Value V = cpf.get(v);
		if (V instanceof ValueZero)
		{
			_spot.setText("0.0");
		}
		else
		{
			_spot.setText(V.getExpr());
		}
		_spot.setFocus();
		_spot.setSelection(0, V.getExpr().length());
	}
	public void commitCPF()
	{
		_spot.setEnabled(false);
		_spot.setVisible(false);
		if (currentCPF != null)
		{
			try
			{
				Value z = new ValueDouble(Double.parseDouble(_spot.getText()));
				//TODO: Hack, need expression parser
				REN._UndoContext.register(new UndoCPFEdit(currentCPF,currentVal));
				currentCPF.put(currentVal, z);
				currentCPF = null;
				currentVal = 0;
			}
			catch (Exception e)
			{}
		}
	}
	public void movespot(int v)
	{
		Rectangle cpfBox = _spot.getBounds();
		int cX = v % resultrendering.Base;
		int cY = v / resultrendering.Base;
		cpfBox.x = cX * resultrendering.cellDim.x + resultrendering.renderStart.x + resultrendering.offSetRect.x + 1;
		cpfBox.y = cY * resultrendering.cellDim.y + resultrendering.renderStart.y + resultrendering.offSetRect.y + 1;
		cpfBox.width = resultrendering.cellDim.x - 2;
		cpfBox.height = resultrendering.cellDim.y - 2;
		_spot.setBounds(cpfBox);
	}
	public void tab()
	{
		if (currentCPF == null) return;
		CPF cpf = currentCPF;
		int v = currentVal;
		commitCPF();
		// must move the box to next cell
		v++;
		v %= cpf.size();
		movespot(v);
		beginCPFedit(cpf, v);
	}
	public void upkey()
	{
		if (currentCPF == null) return;
		CPF cpf = currentCPF;
		int v = currentVal;
		commitCPF();
		// must move the box to next cell
		v -= resultrendering.Base;
		v += cpf.size();
		v %= cpf.size();
		movespot(v);
		beginCPFedit(cpf, v);
	}
	public void downkey()
	{
		if (currentCPF == null) return;
		CPF cpf = currentCPF;
		int v = currentVal;
		commitCPF();
		// must move the box to next cell
		v += resultrendering.Base;
		v %= cpf.size();
		movespot(v);
		beginCPFedit(cpf, v);
	}
	public void shifttab()
	{
		if (currentCPF == null) return;
		CPF cpf = currentCPF;
		int v = currentVal;
		commitCPF();
		// must move the box to next cell
		v--;
		v += cpf.size();
		v %= cpf.size();
		movespot(v);
		beginCPFedit(cpf, v);
	}
	/*
	 public void pan(int dX, int dY)
	 {
	 _XTally += dX;
	 _YTally += dY;
	 if (_SkipFrameCounter <= 0)
	 {
	 _Transform.translate(_XTally, _YTally);
	 ourRedraw();
	 Rectangle R = _spot.getBounds();
	 R.x += _XTally;
	 R.y += _YTally;
	 _XTally = 0;
	 _YTally = 0;
	 _spot.setBounds(R);
	 _SkipFrameCounter = _Constants.TranslationFrameSkip;
	 }
	 _SkipFrameCounter--;
	 }
	 */
	public void setSpotEdit(Text txtedit)
	{
		_spot = txtedit;
		_spot.setEnabled(false);
		_spot.setVisible(false);
	}
	public void event_widgetDisposed(DisposeEvent e)
	{
		killPopUps();
		_Constants.clean();
		if (backbuffer != null) backbuffer.dispose();
		if (back_gc != null) back_gc.dispose();
		// stuff here
	}
	public void SetAllCpf(boolean val)
	{
		REN.setCPFrender(val);
		ourRedraw();
	}
	/*
	public void drawBack(PaintEvent e)
	{
		if (_spot.getEnabled())
		{
			int dX = _Transform.x() - _TranslationTally.x;
			int dY = _Transform.y() - _TranslationTally.y;
			Rectangle R = _spot.getBounds();
			R.x += dX;
			R.y += dY;
			_spot.setBounds(R);
		}
		_TranslationTally.x = _Transform.x();
		_TranslationTally.y = _Transform.y();
		_RenderingContext.gc.setBackground(_Constants.Colors._DefaultBackground);
		_RenderingContext.gc.setForeground(_Constants.Colors._DefaultText);
		//		 erase the background
		_RenderingContext.gc.fillRectangle(0, 0, e.width, e.height);
		// draw the grid
		// TODO: add check for grid
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
			back_gc.drawLine(gX + z, 0, gX + z, e.height);
		}
		for (int z = -dZ; z <= e.height + dZ; z += dZ)
		{
			back_gc.drawLine(0, gY + z, e.width, gY + z);
		}
		_RenderingContext.gc.setBackground(_Constants.Colors._DefaultBackground);
		_RenderingContext.gc.setForeground(_Constants.Colors._DefaultText);
	}
	*/
	public void event_paintControl(PaintEvent e)
	{
		_Transform.setWindow(e.width, e.height);
		if (!super.backbuffer(e)) return;
		// begin rendering on the back plane
		// start the state = initial colors/text
		setupRenderingContext();
		drawBack(e);
		// draw the tree
		REN.Render(_RenderingContext);
		if (_DoDoublePass)
		{
			drawBack(e);
			// draw the tree
			REN.Render(_RenderingContext);
			_DoDoublePass = false;
		}
		if (_OnRenderLoopZoom2Fit && REN._network.getNumberOfVertices() > 0)
		{
			_Transform.normalizescale();
			REN.Render(_RenderingContext);
			_Transform.Zoom2Fit(_RenderingContext.Bounds);
			REN.Render(_RenderingContext);
			_Transform.Center(_RenderingContext.Bounds);
			drawBack(e);
			REN.Render(_RenderingContext);
			//		ourRedraw();
		}
		_OnRenderLoopZoom2Fit = false;
		/*
		 horz.setMinimum(0); horz.setMaximum(RC.Bounds.width);
		 horz.setSelection(_trans.itx(RC.Bounds.x));
		 
		 /*
		 * vert.setMinimum(0); vert.setMaximum(_trans.vert.y - _trans.vert.x);
		 * vert.setSelection(_trans.pan.y - _trans.vert.x);
		 * vert.setVisible(true); vert.setBounds( e.width - 16, e.height/2, 16,
		 * e.height/2 - 16);
		 */
		// draw the cursor
		back_gc.setBackground(_Constants.Colors._DefaultBackground);
		back_gc.setForeground(_Constants.Colors._Cursor);
		back_gc.setLineStyle(_Constants.CursorLineStyle);
		back_gc.drawLine(_Cursor.x - _Constants.CursorLen, _Cursor.y, _Cursor.x + _Constants.CursorLen, _Cursor.y);
		back_gc.drawLine(_Cursor.x, _Cursor.y - _Constants.CursorLen, _Cursor.x, _Cursor.y + _Constants.CursorLen);
		if (_TranslationOn || _DragMode)
		{
			back_gc.setLineStyle(_Constants.TranslateVectorStyle);
			back_gc.setForeground(_Constants.Colors._TranslateVector);
			back_gc.drawLine(_Cursor.x, _Cursor.y, _XCapture, _YCapture);
		}
		e.gc.drawImage(backbuffer, 0, 0);
		drawFinished();
		SyncSliders();
	}
	public void fireLeftClick(int x, int y, boolean shift)
	{
		SelectionContext SC = new SelectionContext();
		SC.x = x;
		SC.y = y;
		SC.multi = shift;
		SC.lookingforEdge = true;
		REN.fireSelelection(SC);
		if (SC.RegIdx >= 0)
		{
			isUtility = SC.foundNode.getType() == BeliefNode.NODE_CHANCE;
			commitCPF();
			_spot.setBounds(SC.Container);
			beginCPFedit(SC.selectedCPF, SC.RegIdx);
			resultrendering = SC.cpfresult;
		}
		else
		{
			commitCPF();
		}
		ourRedraw();
	}
	public void fireRightClick(int x, int y)
	{
		SelectionContext SC = new SelectionContext();
		SC.x = x;
		SC.y = y;
		SC.multi = false;
		SC.lookingforEdge = true;
		REN.fireSelelection(SC);
		_edit = null;
		if (SC.foundNode != null)
		{
			_edit = SC.foundNode;
			menuNode.setVisible(true);
			// need a popup
		}
		if (SC.foundEdge != null)
		{
			_edge = SC.foundEdge;
			menuEdge.setVisible(true);
			// edge popup
		}
		else
		{
			menuNetwork.setVisible(true);
			// network popup
		}
		ourRedraw();
	}
	public void event_mouseDown(MouseEvent e)
	{
		translate_mousedown(e);
	}
	public void event_mouseUp(MouseEvent e)
	{
		if (_LeftMouseDown && !_RightMouseDown && _MouseEventFire == 1 && !edgeMode && !_DragMode)
				fireLeftClick(_Cursor.x, _Cursor.y, (e.stateMask & SWT.SHIFT) > 0);
		if (!_LeftMouseDown && _RightMouseDown && _MouseEventFire == 1 && !edgeMode && !_DragMode)
				fireRightClick(_Cursor.x, _Cursor.y);
		translate_mouseup(e);
		if (edgeMode && _DragMode)
		{
			//System.out.println("creating edge from (" + cursor.x + "," +
			// cursor.y + ") to (" + e.x + "," + e.y + ")");
			SelectionContext SC = new SelectionContext();
			SC.x = _Cursor.x;
			SC.y = _Cursor.y;
			SC.multi = false;
			REN.fireSelelection(SC);
			if (SC.found)
			{
				//System.out.println("from " + SC.foundNode.getName());
				SelectionContext SC2 = new SelectionContext();
				SC2.x = e.x;
				SC2.y = e.y;
				SC2.multi = false;
				REN.fireSelelection(SC2);
				if (SC2.found)
				{
					if (!edgeDo)
						REN.Connect(SC.foundNodeContainer, SC2.foundNodeContainer, SC.foundNode, SC2.foundNode);
					else
						REN.Disconnect(SC.foundNodeContainer, SC2.foundNodeContainer, SC.foundNode, SC2.foundNode);
					//_network.connect( )
					//System.out.println("to " + SC2.foundNode.getName());
					//_network.connect( SC.foundNode, SC2.foundNode);
				}
			}
		}
		endDragMode();
		edgeMode = false;
	}
	public void event_mouseDoubleClick(MouseEvent e)
	{
		SelectionContext SC = new SelectionContext();
		SC.x = e.x;
		SC.y = e.y;
		SC.multi = false;
		REN.fireSelelection(SC);
		if (SC.found)
		{
			// System.out.println("starting the editor!");
			// getShell()
			BeliefNodeEditorDlg bne = new BeliefNodeEditorDlg();
			bne.open(getShell(), _Constants, SC.foundNode, REN._bn, REN._UndoContext);
		}
		_DragMode = false;
		_LeftMouseDown = false;
		// no nothing?
	}
	public void event_mouseMove(MouseEvent e)
	{
		if (enterDragmode())
		{
			commitCPF();
			if (!edgeMode)
			{
				SelectionContext SC = new SelectionContext();
				SC.x = _Cursor.x;
				SC.y = _Cursor.y;
				SC.multi = (e.stateMask & SWT.SHIFT) > 0;
				REN.fireSelelection(SC);
			}
		}
		if (_DragMode)
		{
			int dX = e.x - _XCapture;
			int dY = e.y - _YCapture;
			_XCapture = e.x;
			_YCapture = e.y;
			if (!edgeMode) REN.translate(dX, dY, _Transform);
		}
		DragModeRefresh();
		if (_TranslationOn)
		{
			int dX = e.x - _XCapture;
			int dY = e.y - _YCapture;
			_XCapture = e.x;
			_YCapture = e.y;
			_Transform.translate(dX, dY);
			transformRedraw();
		}
		// if in translation -> translate marginal
	}
	public void event_focusGained(FocusEvent e)
	{
		SendStart();
		mapGainFocus();
	}
	public void event_focusLost(FocusEvent e)
	{
		killMessage();
		mapLostFocus();
	}
	public void event_controlResized(ControlEvent e)
	{
		ourRedraw();
	}
	public void mapCutset()
	{
		BeliefNetwork BND = REN._bn.copy();

		// something bad to original
		//BND.deleteBeliefNode(BND.getNodes()[2]);
		//BND.deleteBeliefNode(BND.getNodes()[0]);
		
		BND = LoopCutset.KC(BND);
		
		REN.ChangeNetworks(BND);
		LayPolyTree.apply(BND.getGraph());
		mapZoom2Fit();
		
		_DoDoublePass = true; // first = flush cache, second = rebuild cache
		ourRedraw();
	}
	public void mapNewNode()
	{
		REN.createnewNode(_Cursor, _Transform, BeliefNode.NODE_CHANCE);
		ourRedraw();
	}
	public void mapNewDescNode()
	{
		REN.createnewNode(_Cursor, _Transform, BeliefNode.NODE_DECISION);
		ourRedraw();
	}
	public void mapNewUtilNode()
	{
		REN.createnewNode(_Cursor, _Transform, BeliefNode.NODE_UTILITY);
		ourRedraw();
	}
	public void mapNewEdge()
	{
		edgeMode = true;
		edgeDo = false;
	}
	public void mapEdgeDelete()
	{
		edgeMode = true;
		edgeDo = true;
	}
	public void mapDeleteNode()
	{
		SelectionContext SC = new SelectionContext();
		SC.x = _Cursor.x;
		SC.y = _Cursor.y;
		SC.multi = false;
		REN.fireSelelection(SC);
		if (SC.found)
		{
			REN.DeleteNode(SC.foundNodeContainer, SC.foundNode);
			ourRedraw();
		}
	}
	public void event_keyPressed(KeyEvent e)
	{
		char echar = e.character;
		// if cpf has focus, route to cpf
		// if cpf has focus + key = tab, go next cell
		if ((e.character == 'z' || e.character == 'Z') && (e.stateMask & SWT.CTRL) > 0)
		{
			undo();
		}
		else if (e.character == 'n' || e.character == 'N')
		{
			mapNewNode();
		}
		else if (e.character == 'b' || e.character == 'B')
		{
			mapNewDescNode();
		}
		else if (e.character == 'v' || e.character == 'V')
		{
			mapNewUtilNode();
		}
		else if (e.character == 'e' || e.character == 'E')
		{
			mapNewEdge();
		}
		else if (e.character == 'd' || e.character == 'D')
		{
			mapEdgeDelete();
		}
		else if (e.character == 'y' || e.character == 'Y')
		{
			Layouts.ArrangeCircle(REN._network, _Constants);
			_OnRenderLoopZoom2Fit = true;
			ourRedraw();
		}
		else if (e.character == 'u' || e.character == 'U')
		{
			Layouts.ArrangeDepthCircleRandom(REN._network, _Constants);
			_OnRenderLoopZoom2Fit = true;
			ourRedraw();
		}
		else if (e.character == 'k' || e.character == 'K')
		{
			mapDeleteNode();
		}
		else if (e.character == 'p' || e.character == 'P')
		{
			LayPolyTree.apply(REN._bn.getGraph());
			_OnRenderLoopZoom2Fit = true;
			ourRedraw();
		}
		else if (e.character == 'w' || e.character == 'W')
		{
			InOut.apply(REN._bn.getGraph());
			ourRedraw();
		}
		else if (e.character == 's' || e.character == 'S')
		{
			Clear.apply(REN._bn.getGraph());
			ourRedraw();
		}
		else if (e.character == 'r' || e.character == 'R')
		{
			LayRandomPolyTree.apply(REN._bn.getGraph());
			_OnRenderLoopZoom2Fit = true;
			ourRedraw();
		}
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
	public RenderNetwork GetRenderNetwork()
	{
		return REN;
	}
	public NetworkEdit(Composite parent, int style)
	{
		//NO_BACKGROUND, NO_REDRAW_RESIZE and NO_MERGE_PAINTS
		//SWT.NO_BACKGROUND
		super(parent, style | SWT.NO_BACKGROUND | SWT.NO_REDRAW_RESIZE | SWT.NO_MERGE_PAINTS);
		REN = new RenderNetwork();
		createPopUps();
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