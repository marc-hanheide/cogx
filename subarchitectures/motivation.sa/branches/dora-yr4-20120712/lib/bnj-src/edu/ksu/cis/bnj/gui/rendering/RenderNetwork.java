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
 */package edu.ksu.cis.bnj.gui.rendering;
import org.eclipse.swt.graphics.Point;
import edu.ksu.cis.bnj.gui.tools.RenderContext;
import edu.ksu.cis.bnj.gui.tools.SelectionContext;
import edu.ksu.cis.bnj.gui.tools.Transformation;
import edu.ksu.cis.bnj.gui.tools.UndoContext;
import edu.ksu.cis.bnj.gui.tools.undo.UndoBatch;
import edu.ksu.cis.bnj.gui.tools.undo.UndoEdgeCreate;
import edu.ksu.cis.bnj.gui.tools.undo.UndoEdgeDelete;
import edu.ksu.cis.bnj.gui.tools.undo.UndoInternalChange;
import edu.ksu.cis.bnj.gui.tools.undo.UndoNodeCreate;
import edu.ksu.cis.bnj.gui.tools.undo.UndoNodeDelete;
import edu.ksu.cis.bnj.gui.tools.undo.UndoWhole;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.inference.exact.LSonline;
import edu.ksu.cis.bnj.ver3.influence.Solver;
import edu.ksu.cis.util.GlobalOptions;
import edu.ksu.cis.util.graph.core.Edge;
import edu.ksu.cis.util.graph.core.Graph;
import edu.ksu.cis.util.graph.core.Vertex;
public class RenderNetwork
{
	public Graph			_network;
	public BeliefNetwork	_bn;
	int						nCnt	= 0;
	public UndoContext		_UndoContext;
	public Transformation	_trans;
	public void Undo()
	{
		_UndoContext.undo();
	}
	public RenderNetwork()
	{
		_network = new Graph();
		_bn = new BeliefNetwork();
		_UndoContext = new UndoContext();
		_UndoContext.Network = this;
	}
	public void ForceDeleteNode(Vertex V, BeliefNode bnode)
	{
		_bn.deleteBeliefNode(bnode);
		_network.removeVertex(V);
	}
	
	public void ChangeNetworks(BeliefNetwork bn)
	{
		UndoWhole u = new UndoWhole(_bn, bn);
		_UndoContext.register(u);
		construct(bn);
	}
	public void DeleteNode(Vertex V, BeliefNode bnode)
	{
		UndoBatch UB = new UndoBatch();
		//UB.Register(new UndoEdgeDelete(A,B,bA,bB));
		Vertex[] C = _network.getChildren(V);
		Vertex[] P = _network.getParents(V);
		UB.Register(new UndoNodeDelete(V, bnode));
		for (int i = 0; i < C.length; i++)
		{
			UB.Register(new UndoEdgeDelete(V, C[i], ((RenderNode) C[i].getObject()).node, bnode));
		}
		//((RenderEditNode) C[i].getObject()).node
		//((RenderEditNode) C[i].getObject()).node
		for (int i = 0; i < P.length; i++)
		{
			UB.Register(new UndoEdgeDelete(P[i], V, bnode, ((RenderNode) P[i].getObject()).node));
		}
		UB.Register(new UndoInternalChange(_bn));
		_UndoContext.register(UB);
		ForceDeleteNode(V, bnode);
	}
	public void ForceCreate(Vertex X, BeliefNode bnode)
	{
		int cx = 0;
		int cy = 0;
		if (bnode.getOwner() != null)
		{
			cx = bnode.getOwner().getx();
			cy = bnode.getOwner().gety();
			bnode.getOwner().setLOC(-1);
		}
		//System.out.println("forcing a create?");
		bnode.setOwner(null);
		_bn.addBeliefNode(bnode);
		X.setLOC(-1);
		_network.addVertex(X);
		bnode.getOwner().translate(cx, cy);
	}
	public void createnewNode(Point p, Transformation T, int type)
	{
		//		Vertex Z = new Vertex("New Node" + nCnt);
		//		Z.translate(p.x,p.y);
		BeliefNode bnode = new BeliefNode("New Node" + nCnt, new Discrete(new String[] { "True", "False" }));
		if(type == BeliefNode.NODE_UTILITY)
			bnode = new BeliefNode("New Node" + nCnt, new Discrete(new String[] { "v" }));
		bnode.setType(type);
		//		_network.addVertex(Z);
		RenderNode ren = new RenderNode(bnode);
		Vertex X = new Vertex("New Node" + nCnt);
		ren.owner = X;
		X.setObject(ren);
		ForceCreate(X, bnode);
		_UndoContext.register(new UndoNodeCreate(X, bnode));
		bnode.getOwner().translate(T.itx(p.x), T.ity(p.y));
		nCnt++;
	}
	public void construct(BeliefNetwork bn)
	{
		_network = new Graph();
		_bn = bn;
		Vertex[] V = bn.getGraph().getVertices();
		Vertex[] L = new Vertex[V.length];
		for (int i = 0; i < V.length; i++)
		{
			//BeliefNode bnode = (BeliefNode) V[i].getObject();
			Vertex X = new Vertex(V[i].getName());
			RenderNode ren = new RenderNode((BeliefNode) V[i].getObject());
			X.setObject(ren);
			L[V[i].loc()] = X;
			_network.addVertex(X);
		}
		for (int i = 0; i < V.length; i++)
		{
			Vertex[] C = bn.getGraph().getChildren(V[i]);
			for (int j = 0; j < C.length; j++)
			{
				_network.addDirectedEdge(L[V[i].loc()], L[C[j].loc()]);
			}
		}
		nCnt = 0;
	}
	private Edge EdgeFound; 
	public void fireSelelection(SelectionContext SC)
	{
		SC.foundEdge = null;
		Vertex[] V = _network.getVertices();
		SC.found = false;
		boolean nodeC = false;
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.fireSelection(SC);
			if (SC.found && !nodeC && !SC.multi)
			{
				SC.foundNodeContainer = V[i];
				nodeC = true;
			}
			if(SC.lookingforEdge)
			{
				int edgeSelect = GlobalOptions.getInstance().getInteger("edge_select_distance", 8);
				Vertex[] C = _network.getChildren(V[i]);
				double vX = _trans.tx(ren.x());
				double vY = _trans.ty(ren.y());
				double mX = SC.x - vX;
				double mY = SC.y - vY;
				for(int j = 0; j < C.length; j++)
				{
					double cX = _trans.tx(((RenderNode) (C[j].getObject())).x()) - vX;
					double cY = _trans.ty(((RenderNode) (C[j].getObject())).y()) - vY;
					double dist = Math.sqrt(cX*cX + cY*cY); 
					double lX = cX / dist;
					double lY = cY / dist;
					double nX = -lY;
					double nY =  lX;
					
					double vProj = mX * lX + mY * lY;
					if(dist / 3 <= vProj && vProj <= 2 * dist / 3)
					{
						double nProj = mX * nX + mY * nY;
						if(-edgeSelect <= nProj && nProj <= edgeSelect)
						{
							SC.foundEdge = new Edge(V[i],C[j]);
						}
					}
				}
			}
		}
		EdgeFound = SC.foundEdge;
		
	}
	public boolean	mark[];
	public boolean canGo(Vertex A, Vertex B)
	{
		if (mark[A.loc()]) return false;
		mark[A.loc()] = true;
		Vertex[] Children = _network.getChildren(A);
		for (int i = 0; i < Children.length; i++)
		{
			if (Children[i].loc() == B.loc()) return true;
			if (canGo(Children[i], B)) return true;
		}
		return false;
	}
	public void ForceConnect(Vertex A, Vertex B, BeliefNode bA, BeliefNode bB)
	{
		_bn.connect(bA, bB);
		_network.addDirectedEdge(A, B);
	}
	public void Connect(Vertex A, Vertex B, BeliefNode bA, BeliefNode bB)
	{
		if(bA.getType() == BeliefNode.NODE_UTILITY) return;
		
		if (_network.getConnectedness(A, B) != 0) return;
		Vertex[] V = _network.getVertices();
		mark = new boolean[V.length];
		for (int i = 0; i < V.length; i++)
			mark[i] = false;
		if (canGo(B, A)) return;
		_UndoContext.register(new UndoEdgeCreate(A, B, bA, bB));
		ForceConnect(A, B, bA, bB);
	}
	// undo done!
	public void Disconnect(Vertex A, Vertex B, BeliefNode bA, BeliefNode bB)
	{
		if (_network.getConnectedness(A, B) < 0) return;
		UndoBatch UB = new UndoBatch();
		UB.Register(new UndoEdgeDelete(A, B, bA, bB));
		UB.Register(new UndoInternalChange(_bn));
		_UndoContext.register(UB);
		ForceDisconnect(A, B, bA, bB);
	}
	public void ForceDisconnect(Vertex A, Vertex B, BeliefNode bA, BeliefNode bB)
	{

		
		_bn.disconnect(bA, bB);
		_bn.disconnect(bB, bA);
		_network.removeEdge(A, B);
	}
	public void Render(RenderContext RC)
	{
		_trans = RC.Trans;
		RC.Bounds.x = 100000;
		RC.Bounds.y = 100000;
		RC.Bounds.width = -100000;
		RC.Bounds.height = -100000;
		Vertex[] V = _network.getVertices();
		RC.gc.setForeground(RC.constants.Colors._Edge_Color);
		RC.gc.setLineWidth(RC.constants.EdgeWidth);
		RC.gc.setLineStyle(RC.constants.EdgeStyle);
		RC.gc.setFont(RC.constants.Fonts.RoadMap);
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			Vertex[] C = _network.getChildren(V[i]);
			for (int j = 0; j < C.length; j++)
			{
				RenderNode cren = (RenderNode) (C[j].getObject());
				if(EdgeFound == null)
				{
					if(cren.node.getType()==BeliefNode.NODE_CHANCE)
					{
						RenderEdge.rRenderEdge(ren, cren, RC);
					}
					else if(cren.node.getType()==BeliefNode.NODE_DECISION)
					{
						RenderEdge.rRenderEdgeD(ren, cren, RC);
					}
					else if(cren.node.getType()==BeliefNode.NODE_UTILITY)
					{
						RenderEdge.rRenderEdgeU(ren, cren, RC);
					}
				}
				else
				{
					if(V[i]==EdgeFound.s() && C[j]==EdgeFound.d())
					{
						RC.gc.setForeground(RC.constants.Colors._Edge_Color_Selected);
						RC.gc.setLineWidth(RC.constants.EdgeWidthSelected);
						RC.gc.setLineStyle(RC.constants.EdgeStyleSelected);
						if(cren.node.getType()==BeliefNode.NODE_CHANCE)
						{
							RenderEdge.rRenderEdge(ren, cren, RC);
						}
						else if(cren.node.getType()==BeliefNode.NODE_DECISION)
						{
							RenderEdge.rRenderEdgeD(ren, cren, RC);
						}
						else if(cren.node.getType()==BeliefNode.NODE_UTILITY)
						{
							RenderEdge.rRenderEdgeU(ren, cren, RC);
						}
						RC.gc.setForeground(RC.constants.Colors._Edge_Color);
						RC.gc.setLineWidth(RC.constants.EdgeWidth);
						RC.gc.setLineStyle(RC.constants.EdgeStyle);
					}
					else
					{
						if(cren.node.getType()==BeliefNode.NODE_CHANCE)
						{
							RenderEdge.rRenderEdge(ren, cren, RC);
						}
						else if(cren.node.getType()==BeliefNode.NODE_DECISION)
						{
							RenderEdge.rRenderEdgeD(ren, cren, RC);
						}
						else if(cren.node.getType()==BeliefNode.NODE_UTILITY)
						{
							RenderEdge.rRenderEdgeU(ren, cren, RC);
						}
					}
				}
			}
		}
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.rRenderNode(RC);
		}
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.RenderCPF(RC);
		}
		// check to see if render bounding box
		RC.Bounds.width -= RC.Bounds.x;
		RC.Bounds.height -= RC.Bounds.y;
		if (RC.constants.RenderNetworkBoundingBox)
		{
			RC.gc.drawRectangle(RC.Bounds);
		}
	}
	public void translate(int dx, int dy, Transformation T)
	{
		Vertex[] V = _network.getVertices();
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.translate(dx, dy, T);
		}
	}
	public void Switch2Edit()
	{
		Vertex[] V = _network.getVertices();
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.switch2edit();
		}
	}
	public void Switch2Run(Solver solve)
	{
		Vertex[] V = _network.getVertices();
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.switch2run(solve);
		}
	}
	public void setCPFrender(boolean val)
	{
		Vertex[] V = _network.getVertices();
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.renderCpf = val;
		}
	}
	public void snap2grid(int grid)
	{
		Vertex[] V = _network.getVertices();
		for (int i = 0; i < V.length; i++)
		{
			RenderNode ren = (RenderNode) (V[i].getObject());
			ren.snap2grid(grid);
		}
	}
	public String getName()
	{
		return "Network";
	}
}