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
import java.text.DecimalFormat;
import org.eclipse.swt.graphics.GC;
import org.eclipse.swt.graphics.Point;
import edu.ksu.cis.bnj.gui.tools.ColorWheel;
import edu.ksu.cis.bnj.gui.tools.Constants;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.CPF;
import edu.ksu.cis.bnj.ver3.core.Value;
import edu.ksu.cis.bnj.ver3.core.values.ValueDouble;
import edu.ksu.cis.bnj.ver3.core.values.ValueZero;
import org.eclipse.swt.graphics.*;
public class RenderCPF
{
	public Rectangle	offSetRect	= new Rectangle(0, 0, 1, 1);
	public Point		cellDim		= new Point(1, 1);
	public Point		renderStart	= new Point(1, 1);
	public int			Base;

	static public RenderCPF rCPF(GC gc, int x, int y, CPF cpf, Constants constants, boolean prob)
	{
		ColorWheel color = constants.Colors;
		gc.setFont(constants.Fonts.CPF_Body);
		RenderCPF finalResult = new RenderCPF();
		BeliefNode[] dom = cpf.getDomainProduct();
		int gWidth = 0;
		int mult = 1;
		int initOffX = 0;
		int initMaxY = 0;
		int zWidthBottom;
		Point maxValueSize = gc.textExtent("1100%%");
		for (int i = 0; i < dom[0].getDomain().getOrder(); i++)
		{
			Point size = gc.textExtent(dom[0].getDomain().getName(i));
			if (initOffX < size.x) initOffX = size.x;
			if (initMaxY < size.y) initMaxY = size.y;
		}
		for (int i = 1; i < dom.length; i++)
		{
			String str = dom[i].getName();
			Point size = gc.textExtent(str);
			if (initOffX < size.x) initOffX = size.x;
		}
		initOffX += constants.CPF_CellSpacing;
		initMaxY += constants.CPF_CellSpacing;
		for (int i = dom.length - 1; i > 0; i--)
		{
			int K = dom[i].getDomain().getOrder();
			for (int j = 0; j < K; j++)
			{
				Point size = gc.textExtent(dom[i].getDomain().getName(j));
				if (size.x + 5 > gWidth)
				{
					gWidth = size.x + constants.CPF_CellPadding;
				}
			}
			gWidth *= K;
			mult *= K;
		}
		zWidthBottom = gWidth / mult;
		if (maxValueSize.x > zWidthBottom) zWidthBottom = maxValueSize.x + 5;
		int offY = initMaxY * (dom.length - 1);
		gc.setBackground(color._CPF_Background_Header);
		gc.setForeground(color._CPF_Text_Header);
		gc.fillRectangle(x, y, zWidthBottom * mult + initOffX, offY);
		gc.fillRectangle(x, y + offY, initOffX, initMaxY * dom[0].getDomain().getOrder());
		gc.setBackground(color._DefaultBackground);
		gc.fillRectangle(x + initOffX, y + offY, zWidthBottom * mult, initMaxY * dom[0].getDomain().getOrder());
		gc.setBackground(color._CPF_Background_Header);
		int multB = mult;
		mult = 1;
		offY = 0;
		for (int i = 1; i < dom.length; i++)
		{
			int K = dom[i].getDomain().getOrder();
			int zWidth = zWidthBottom * multB / K;
			int off = 0;
			for (int j = 0; j < mult; j++)
			{
				for (int k = 0; k < K; k++)
				{
					String str = dom[i].getDomain().getName(k);
					Point size = gc.textExtent(str);
					int offX = (zWidth - size.x) / 2;
					gc.drawString(str, initOffX + x + offX + zWidth * off, y + offY + constants.CPF_CellPadding / 2);
					gc.drawLine(initOffX + x + zWidth * off, y + offY, initOffX + x + zWidth * (off + 1), y + offY);
					gc
							.drawLine(initOffX + x + zWidth * off, y + offY, initOffX + x + zWidth * off, y + initMaxY
									+ offY);
					gc.drawLine(initOffX + x + zWidth * (off + 1), y + offY, initOffX + x + zWidth * (off + 1), y
							+ initMaxY + offY);
					off++;
				}
			}
			mult *= K;
			multB /= K;
			offY += initMaxY;
		}
		int oOffY = offY;
		offY = 0;
		for (int i = 1; i < dom.length; i++)
		{
			String str = dom[i].getName();
			Point size = gc.textExtent(str);
			int offX = (initOffX - size.x) / 2;
			gc.drawString(str, x + offX, y + offY + constants.CPF_CellPadding / 2);
			gc.drawLine(x, y + offY, x, y + offY + initMaxY);
			gc.drawLine(x, y + offY, initOffX + x, y + offY);
			offY += initMaxY;
		}
		offY = oOffY;
		for (int i = 0; i < dom[0].getDomain().getOrder(); i++)
		{
			String str = dom[0].getDomain().getName(i);
			Point size = gc.textExtent(dom[0].getDomain().getName(i));
			int offX = (initOffX - size.x) / 2;
			gc.drawString(str, x + offX, y + offY + constants.CPF_CellPadding / 2);
			gc.drawLine(x, y + offY, x, y + offY + initMaxY);
			gc.drawLine(x, y + offY, initOffX + x, y + offY);
			offY += initMaxY;
		}
		gc.setBackground(color._DefaultBackground);
		gc.setForeground(color._DefaultText);
		gc.setFont(constants.Fonts.CPF_Body);
		offY = oOffY;
		int cpfV = 0;
		DecimalFormat format = new DecimalFormat("##0.0'%'"); // $NON-NLS-1$
		DecimalFormat format2 = new DecimalFormat("##0.0"); // $NON-NLS-1$
		for (int i = 0; i < dom[0].getDomain().getOrder(); i++)
		{
			String str = dom[0].getDomain().getName(i);
			Point size = gc.textExtent(dom[0].getDomain().getName(i));
			int offX = (initOffX - size.x) / 2;
			for (int off = 0; off < mult; off++)
			{
				Value v = cpf.get(cpfV);
				String str2 = v.getExpr();
				if (v instanceof ValueDouble && prob)
				{
					gc.setForeground(color._CPF_Text);
					double percent = ((ValueDouble) v).getValue() * 100;
					str2 = format.format(percent);
				}
				if (v instanceof ValueDouble && !prob)
				{
					gc.setForeground(color._CPF_Text);
					double val = ((ValueDouble) v).getValue();
					str2 = format2.format(val);
				}
				else
				{
					if (v instanceof ValueZero)
					{
						gc.setForeground(color._InvalidCPFValue);
						str2 = "0.0";
					}
				}
				Point size2 = gc.textExtent(str2);
				int offX2 = (zWidthBottom - size2.x) / 2;
				gc
						.drawString(str2, x + offX2 + zWidthBottom * off + initOffX, y + offY
								+ constants.CPF_CellPadding / 2);
				gc.setForeground(color._DefaultText);
				gc.drawLine(initOffX + x + zWidthBottom * off, y + offY, initOffX + x + zWidthBottom * (off + 1), y
						+ offY);
				gc.drawLine(initOffX + x + zWidthBottom * off, y + offY, initOffX + x + zWidthBottom * off, y + offY
						+ initMaxY);
				gc.drawLine(initOffX + x + zWidthBottom * (off + 1), y + offY, initOffX + x + zWidthBottom * (off + 1),
						y + initMaxY + offY);
				cpfV++;
			}
			offY += initMaxY;
		}
		gc.drawLine(x, y + offY, x + zWidthBottom * mult + initOffX, y + offY);
		finalResult.offSetRect.x = initOffX;
		finalResult.offSetRect.y = oOffY;
		finalResult.offSetRect.width = zWidthBottom * mult;
		finalResult.offSetRect.height = offY;
		finalResult.cellDim.x = zWidthBottom;
		finalResult.cellDim.y = initMaxY;
		finalResult.renderStart.x = x;
		finalResult.renderStart.y = y;
		finalResult.Base = (finalResult.offSetRect.width / finalResult.cellDim.x);
		return finalResult;
	}
	public String getName()
	{
		return "CPF";
	}
}
//cpf_backcolor
//		gc.setForeground(cpf_backcolor);
/*
 * sizeZ = gc.textExtent (" <expr ...>"); if(sizeZ.x > zWidthBottom)
 * zWidthBottom = sizeZ.x + 16;
 */
/*
 * for(int i = 0; i < _currentCPF.size(); i++) { Value v = _currentCPF.get(i);
 * String z = v.getExpr(); Point size = gc.textExtent (z); if(size.y >
 * zWidthBottom ) zWidthBottom = size.y + 16; }
 */
//gc.drawString(str,x+offX,y+offY+3);
//gc.drawLine(x,y+offY,x,y+offY+initMaxY);
//gc.drawLine(x,y+offY,initOffX+x,y+offY);
/*
 * gc.setFont(constants.Fonts.CPF_Header); gc.drawText(dom[0].getName(), x+10,
 * y-30); int wid = gc.textExtent(dom[0].getName()).x + 20; int wid2 =
 * initOffX+zWidthBottom*mult; if(wid2 >= wid) wid = wid2;
 * gc.drawRoundRectangle(x-10,y-40,wid+20, offY+50,9,9);
 */