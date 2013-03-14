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
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Rectangle;
import edu.ksu.cis.bnj.gui.tools.Constants;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.inference.exact.LSonline;
import edu.ksu.cis.bnj.ver3.influence.Solver;
import org.eclipse.swt.graphics.Point;
import edu.ksu.cis.bnj.gui.tools.RenderContext;
import edu.ksu.cis.bnj.gui.tools.SelectionContext;
import edu.ksu.cis.bnj.gui.tools.Transformation;
import edu.ksu.cis.util.graph.core.Vertex;
public class RenderNode
{
	public Vertex			owner;
	public boolean			renderCpf;
	private boolean			renderMarginal	= false;
	private boolean			selected;
	public BeliefNode		node;
	public Point			size;
	private Point			pos;
	private double			errX;
	private double			errY;
	private Transformation	Tr;
	double					cpf_x;
	double					cpf_y;
	int						cpf_d;
	RenderCPF				resultrendering;
	Rectangle				cpfBox			= new Rectangle(0, 0, 1, 1);
	
	
	private boolean isxyin(int x, int y)
	{
		double lX = x - Tr.tx(x()) + size.x / 2;
		double lY = y - Tr.ty(y()) + size.y / 2;
		if(node.getType() == BeliefNode.NODE_CHANCE)
		{
			if (lX < 0 || lY < 0) return false;
			if (lX >= size.x || lY >= size.y) return false;
			lX -= size.x / 2;
			lY -= size.y / 2;
			lX /= (size.x / 2);
			lY /= (size.y / 2);
			double L = Math.sqrt(lX * lX + lY * lY);
			if (L > 1) return false;
			return true;
		}
		if(node.getType() == BeliefNode.NODE_DECISION)
		{
			if (lX < 0 || lY < 0) return false;
			if (lX >= size.x || lY >= size.y) return false;
			return true;
		}
		if(node.getType() == BeliefNode.NODE_UTILITY)
		{
			if (lX < 0 || lY < 0) return false;
			if (lX >= size.x || lY >= size.y) return false;

			double mx = Math.abs(lX - size.x/2);
			double my = Math.abs(lY - size.y/2);
			if(size.y/2 - mx * size.y/size.x < my) return false;
//			if( Math.abs(lX * lY) > size.x*size.y/8) return false;
			//if ((size.y/2 - (lX - size.x/2)* size.y/size.x) > lY) return false;
			//if (Math.abs(size.y/2 - (size.x - lX) * size.y/size.x) > Math.abs(lY)) return false;
			return true;
		}
		return false;
	}
	
	private boolean isxyin_cpf_enable(int px, int py)
	{
		double x_cpf = px - (int) (Tr.tx(x()) + cpf_x * size.x / 2.0);
		double y_cpf = py - (int) (Tr.ty(y()) + cpf_y * size.y / 2.0);
		if (x_cpf < 0 || y_cpf < 0) return false;
		if (x_cpf > cpf_d || y_cpf > cpf_d) return false;
		return true;
	}
	private int isxyin_cpf(int px, int py)
	{
		double x_cpf = px - (resultrendering.renderStart.x + resultrendering.offSetRect.x);
		double y_cpf = py - (resultrendering.renderStart.y + resultrendering.offSetRect.y);
		if (x_cpf < 0 || y_cpf < 0) return -1;
		if (x_cpf > resultrendering.offSetRect.width || y_cpf > resultrendering.offSetRect.height) return -1;
		int cX = (int) (x_cpf / resultrendering.cellDim.x);
		int cY = (int) (y_cpf / resultrendering.cellDim.y);
		cpfBox.x = cX * resultrendering.cellDim.x + resultrendering.renderStart.x + resultrendering.offSetRect.x + 1;
		cpfBox.y = cY * resultrendering.cellDim.y + resultrendering.renderStart.y + resultrendering.offSetRect.y + 1;
		cpfBox.width = resultrendering.cellDim.x - 2;
		cpfBox.height = resultrendering.cellDim.y - 2;
		return cX + cY * (resultrendering.offSetRect.width / resultrendering.cellDim.x);
	}
	private boolean isxyin_cpfheader(int px, int py)
	{
		if (resultrendering == null) return false;
		double x_cpf = px - (resultrendering.renderStart.x );
		double y_cpf = py - (resultrendering.renderStart.y );
		if (x_cpf < 0 || y_cpf < 0) return false;
		if (x_cpf > resultrendering.offSetRect.width + resultrendering.offSetRect.x
				|| y_cpf >= resultrendering.offSetRect.height + resultrendering.offSetRect.y) return false;
		return true;
	}
	public double x()
	{
		return pos.x + errX;
	}
	public double y()
	{
		return pos.y + errY;
	}
	public void fireSelection(SelectionContext SC)
	{
		if (!SC.multi) selected = false;
		if (isxyin_cpf_enable(SC.x, SC.y) && (node.getType() != BeliefNode.NODE_DECISION || renderMarginal))
		{
			renderCpf = !renderCpf;
			return;
		}
		if (renderCpf && (node.getType() != BeliefNode.NODE_DECISION || renderMarginal) )
		{
			int c = isxyin_cpf(SC.x, SC.y);
			if (c >= 0)
			{
				if (renderMarginal)
				{
					if (c < node.getDomain().getOrder())
					{
						SC.selectedCPF = _Solver.queryMarginal(node);
						SC.foundNode = node;
						SC.found = true;
						SC.RegIdx = c;
						SC.Container = cpfBox;
						SC.cpfresult = resultrendering;
					}
				}
				else
				{
					SC.foundNode = node;
					SC.found = true;

					SC.selectedCPF = node.getCPF();
					SC.RegIdx = c;
					SC.Container = cpfBox;
					SC.cpfresult = resultrendering;
				}
				return;
			}
		}
		if (SC.found && !SC.multi) return;
		if (isxyin(SC.x, SC.y) || isxyin_cpfheader(SC.x, SC.y))
		{
			selected = true;
			if (selected && !SC.multi) SC.foundNode = node;
			SC.found = true;
		}
	}
	public void translate(int dx, int dy, Transformation T)
	{
		if (T == null)
		{
			Vertex s = node.getOwner();
			s.translate(dx, dy);
			pos.x = s.getx();
			pos.y = s.gety();
			return;
		}
		if (!selected) return;
		Vertex s = node.getOwner();
		double ldx = T.is(dx);
		double ldy = T.is(dy);
		errX += ldx - (int) ldx;
		errY += ldy - (int) ldy;
		int eX = (int) errX;
		int eY = (int) errY;
		ldx += eX;
		ldy += eY;
		errX -= eX;
		errY -= eY;
		s.translate((int) ldx, (int) ldy);
		pos.x = s.getx();
		pos.y = s.gety();
	}
	
	public void snap2grid(int grid)
	{
		Vertex s = node.getOwner();
		int fX = s.getx();
		int fY = s.gety();
		
		int dX = (fX % grid);
		int dY = (fY % grid);
		
		s.translate( (dX < grid/2 ? -dX : (grid - dX) ), (dY < grid/2 ? -dY : (grid - dY) ) );
		pos.x = s.getx();
		pos.y = s.gety();
		errX = 0;
		errY = 0;
	}
	
	public void move(int x, int y)
	{
		Vertex s = node.getOwner();
		s.translate(-s.getx() + x, -s.gety() + y);
		pos.x = s.getx();
		pos.y = s.gety();
	}
	public RenderNode(BeliefNode bnode)
	{
		node = bnode;
		renderCpf = false;
		pos = new Point(0, 0);
		size = new Point(0, 0);
		selected = false;
	}
	public void rRenderNode(RenderContext RC)
	{
		Constants constants = RC.constants;
		Transformation T = RC.Trans;
		GC gc = RC.gc;
		Vertex s = node.getOwner();
		pos.x = s.getx();
		pos.y = s.gety();
		size = RenderNode.rBefiefNode(gc, T.tx(x()), T.ty(y()), node, node.getOwner(), constants, selected);
		
		if(node.getType() != BeliefNode.NODE_DECISION || renderMarginal)
		{
			int x_cpf = (int) (T.tx(x()) + constants.EnableCPF_Xf * size.x / 2.0);
			int y_cpf = (int) (T.ty(y()) + constants.EnableCPF_Yf * size.y / 2.0);
			cpf_x = constants.EnableCPF_Xf;
			cpf_y = constants.EnableCPF_Yf;
			cpf_d = constants.EnableCPF_D;
			gc.setBackground(constants.Colors._EnableCPF_Back);
			gc.setForeground(constants.Colors._EnableCPF_Border);
			gc.fillRoundRectangle(x_cpf, y_cpf, constants.EnableCPF_D, constants.EnableCPF_D, 5, 5);
			gc.drawRoundRectangle(x_cpf, y_cpf, constants.EnableCPF_D, constants.EnableCPF_D, 5, 5);
			gc.setForeground(constants.Colors._EnableCPF_Text);
			int z2 = 0;
			int dz2 = 2;
			if (renderCpf)
			{
				z2 = constants.EnableCPF_D - 2;
				dz2 = -2;
			}
			for (int z = 2; z < constants.EnableCPF_D / 2; z++)
			{
				gc.drawLine(x_cpf + z, y_cpf + 1 + z2, x_cpf + constants.EnableCPF_D - z, y_cpf + 1 + z2);
				z2 += dz2;
			}
		}
		int minX = T.tx(x()) - size.x / 2;
		int minY = T.ty(y()) - size.y / 2;
		int maxX = T.tx(x()) + size.x / 2;
		int maxY = T.ty(y()) + size.y / 2;
		if (RC.Bounds.width < maxX) RC.Bounds.width = maxX;
		if (RC.Bounds.height < maxY) RC.Bounds.height = maxY;
		if (RC.Bounds.x > minX) RC.Bounds.x = minX;
		if (RC.Bounds.y > minY) RC.Bounds.y = minY;
	}
	public void RenderCPF(RenderContext RC)
	{
		Constants constants = RC.constants;
		Transformation T = RC.Trans;
		GC gc = RC.gc;
		Tr = T;
		resultrendering = null;
		if (renderCpf && (node.getType() != BeliefNode.NODE_DECISION || renderMarginal))
		{
			if (renderMarginal)
			{
				CPF marg = _Solver.queryMarginal(node);
				resultrendering = RenderCPF.rCPF(gc, T.tx(x()), T.ty(y()) + size.y / 3, marg, constants, node.getType() == BeliefNode.NODE_CHANCE);
			}
			else
			{
				resultrendering = RenderCPF.rCPF(gc, T.tx(x()), T.ty(y()) + size.y / 3, node.getCPF(), constants, node.getType() == BeliefNode.NODE_CHANCE);
			}
			int maxX = resultrendering.renderStart.x + resultrendering.offSetRect.x + resultrendering.offSetRect.width;
			int maxY = resultrendering.renderStart.y + resultrendering.offSetRect.y + resultrendering.offSetRect.height;
			int minX = resultrendering.renderStart.x;
			int minY = resultrendering.renderStart.y;
			if (RC.Bounds.width < maxX) RC.Bounds.width = maxX;
			if (RC.Bounds.height < maxY) RC.Bounds.height = maxY;
			if (RC.Bounds.x > minX) RC.Bounds.x = minX;
			if (RC.Bounds.y > minY) RC.Bounds.y = minY;
		}
	}
	public String getName()
	{
		return node.getName();
	}
	static public Point rBefiefNode(GC gc, int x, int y, BeliefNode bnode, Vertex O, Constants constants, boolean isselected)
	{
		gc.setFont(constants.Fonts.Node_Body);
		Point size = gc.textExtent(bnode.getName());
		//size.y *= 1.4;
		if (!isselected)
		{
			gc.setBackground(constants.Colors._NodeBackColor[O.getAttrib(2)]);
		}
		else
		{
			gc.setBackground(constants.Colors._Node_Background_Selected);
		}
		gc.setLineWidth(2);
		int aW = 20;
		int aH = 20;
		if(bnode.getType() == BeliefNode.NODE_UTILITY)
		{
			aW += 20; // need this such to solve eq for line
			aH += 20; // \todo dito
		}
		int width = size.x + aW;
		int height = (int) (size.y) + aH;
		
		Point dim = new Point(width, height);
		if(bnode.getType() == BeliefNode.NODE_CHANCE)
		{
			if (!isselected)
			{
				gc.setBackground(constants.Colors._NodeBackColor[O.getAttrib(2)]);
				gc.setForeground(constants.Colors._Node_Border);
			}
			else
			{
				gc.setBackground(constants.Colors._Node_Background_Selected);
				gc.setForeground(constants.Colors._Node_Border_Selected);
			}
			gc.fillRoundRectangle(x - width / 2, y - height / 2, width, height, width, height);
			gc.drawRoundRectangle(x - width / 2, y - height / 2, width, height, width, height);
		}
		else if(bnode.getType() == BeliefNode.NODE_DECISION)
		{
			if (!isselected)
			{
				gc.setBackground(constants.Colors._Desc_Color);
				gc.setForeground(constants.Colors._Node_Border);
			}
			else
			{
				gc.setBackground(constants.Colors._Node_Background_Selected);
				gc.setForeground(constants.Colors._Node_Border_Selected);
			}
			gc.fillRectangle(x - width / 2, y - height / 2, width, height);
			gc.drawRectangle(x - width / 2, y - height / 2, width, height);
		}
		else if(bnode.getType() == BeliefNode.NODE_UTILITY)
		{
			if (!isselected)
			{
				gc.setBackground(constants.Colors._Util_Color);
				gc.setForeground(constants.Colors._Node_Border);
			}
			else
			{
				gc.setBackground(constants.Colors._Node_Background_Selected);
				gc.setForeground(constants.Colors._Node_Border_Selected);
			}
			int[] coord = new int[] {x            , y + height / 2, 
	                 x + width / 2, y, 
					 x            , y - height / 2,
					 x - width / 2, y};
			gc.fillPolygon(coord);
			gc.drawPolygon(coord);
			/*
			gc.fillRectangle(x - width / 2, y - height / 2, width, height);
			if (!isselected)
			{
				gc.setBackground(constants.Colors._NodeBackColor[O.getAttrib(2)]);
				gc.setForeground(constants.Colors._Node_Border);
			}
			else
			{
				gc.setBackground(constants.Colors._Node_Background_Selected);
				gc.setForeground(constants.Colors._Node_Border_Selected);
			}
			gc.drawRectangle(x - width / 2, y - height / 2, width, height);
			*/
		}
		gc.setLineWidth(1);
		gc.setForeground(constants.Colors._NodeForeColor[O.getAttrib(2)]);
		gc.drawText(bnode.getName(), x + aW/2 - width / 2, y - size.y / 2);
		return dim;
	}
	public void switch2edit()
	{
		renderMarginal = false;
	}
	private Solver _Solver;
	public void switch2run(Solver solve)
	{
		renderMarginal = true;
		_Solver = solve;
	}
}