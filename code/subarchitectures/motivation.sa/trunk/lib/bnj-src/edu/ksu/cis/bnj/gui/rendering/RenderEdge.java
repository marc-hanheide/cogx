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
import org.eclipse.swt.graphics.Point;
import edu.ksu.cis.bnj.gui.tools.Constants;
import edu.ksu.cis.bnj.gui.tools.RenderContext;
import edu.ksu.cis.bnj.gui.tools.Transformation;
public class RenderEdge
{
	public static void rRenderEdge2bi(int aX, int aY, int bX, int bY, RenderContext RC, Point s)
	{
		GC gc = RC.gc;
		gc.drawLine(aX, aY, bX, bY);
	}

	public static void rRenderEdge2(int aX, int aY, int bX, int bY, RenderContext RC, Point s)
	{
		Constants constants = RC.constants;
		Transformation T = RC.Trans;
		GC gc = RC.gc;
		double zX = (aX - bX);
		double zY = (aY - bY);
		double rX = s.x / 2;
		double rY = s.y / 2;
		double dX = zX / rX;
		double dY = zY / rY;
		double L = Math.sqrt(dX * dX + dY * dY);
		dX /= L;
		dY /= L;
		dX *= rX;
		dY *= rY;
		double eX = dX;
		double eY = dY;
		double L2 = Math.sqrt(eX * eX + eY * eY);
		eX /= L2;
		eY /= L2;
		double rotX = 0.86602540378443864676372317075294;
		double rotY = 0.5;
		int len = constants.ArrowLen;
		double fX = (rotX * eX - rotY * eY) * len;
		double fY = (rotY * eX + rotX * eY) * len;
		int gX = bX + (int) (dX + fX);
		int gY = bY + (int) (dY + fY);
		double hX = (rotX * eX + rotY * eY) * len;
		double hY = (-rotY * eX + rotX * eY) * len;
		int iX = bX + (int) (dX + hX);
		int iY = bY + (int) (dY + hY);
		int cX = bX + (int) dX;
		int cY = bY + (int) dY;
		gc.drawLine(aX, aY, bX, bY);
//		gc.drawLine(cX, cY, gX, gY);
//		gc.drawLine(cX, cY, iX, iY);
		
		
		
		//new int[]{cX,gX,iX},new int[]{cY,gY,iY}
		//gc.drawPolygon(new int[]{cX,cY,gX,gY,iX,iY});
		RC.gc.setBackground(RC.constants.Colors._Edge_Color);
		gc.fillPolygon(new int[]{cX,cY,gX,gY,iX,iY});
	}
	
	public  static void rRenderEdge(RenderNode src, RenderNode dest, RenderContext RC)
	{
		Constants constants = RC.constants;
		Transformation T = RC.Trans;
		GC gc = RC.gc;
		int aX = T.tx(src.x());
		int aY = T.ty(src.y());
		int bX = T.tx(dest.x());
		int bY = T.ty(dest.y());
		Point s = dest.size;
		double zX = (aX - bX);
		double zY = (aY - bY);
		double rX = s.x / 2;
		double rY = s.y / 2;
		double dX = zX / rX;
		double dY = zY / rY;
		double L = Math.sqrt(dX * dX + dY * dY);
		dX /= L;
		dY /= L;
		dX *= rX;
		dY *= rY;
		double eX = dX;
		double eY = dY;
		double L2 = Math.sqrt(eX * eX + eY * eY);
		eX /= L2;
		eY /= L2;
		double rotX = 0.86602540378443864676372317075294;
		double rotY = 0.5;
		int len = constants.ArrowLen;
		double fX = (rotX * eX - rotY * eY) * len;
		double fY = (rotY * eX + rotX * eY) * len;
		int gX = bX + (int) (dX + fX);
		int gY = bY + (int) (dY + fY);
		double hX = (rotX * eX + rotY * eY) * len;
		double hY = (-rotY * eX + rotX * eY) * len;
		int iX = bX + (int) (dX + hX);
		int iY = bY + (int) (dY + hY);
		int cX = bX + (int) dX;
		int cY = bY + (int) dY;
		double midX1 = aX;
		double midY1 = aY;
		double midX2 = bX;
		double midY2 = bY;
		double L4 = Math.sqrt(zX * zX + zY * zY);
		if (L4 >= constants.RoadMapLength)
		{
			if (midX1 < 0)
			{
				midY1 = (aX / dX) * (-dY) + aY;
				midX1 = 0;
			}
			else if (midX1 > T.w())
			{
				midY1 = ((T.w() - bX) / dX) * dY + bY;
				midX1 = T.w();
			}
			if (midX2 < 0)
			{
				midY2 = (bX / dX) * (-dY) + bY;
				midX2 = 0;
			}
			else if (midX2 > T.w())
			{
				midY2 = ((T.w() - aX) / dX) * dY + aY;
				midX2 = T.w();
			}
			if (midY1 < 0)
			{
				midX1 = (aY / dY) * (-dX) + aX;
				midY1 = 0;
			}
			if (midY2 < 0)
			{
				midX2 = (bY / dY) * (-dX) + bX;
				midY2 = 0;
			}
			if (midY1 > T.h())
			{
				midX1 = ((T.h() - bY) / dY) * (dX) + bX;
				midY1 = T.h();
			}
			if (midY2 > T.h())
			{
				midX2 = ((T.h() - aY) / dY) * (dX) + aX;
				midY2 = T.h();
			}
			double kX = midX2 - midX1;
			double kY = midY2 - midY1;
			double L3 = Math.sqrt(kX * kX + kY * kY);
			if (L3 >= 2)
			{
				RC.gc.setBackground(RC.constants.Colors._DefaultBackground);
				
				double u1 = constants.RoadMapRatio * L3;
				double u2 = constants.RoadMapRatioArrow * L3;
				int mX = (int) ((midX1 + midX2) / 2 + eX * u1);
				int mY = (int) ((midY1 + midY2) / 2 + eY * u1);
				int nX = (int) ((midX1 + midX2) / 2 - eX * u1);
				int nY = (int) ((midY1 + midY2) / 2 - eY * u1);
				int pX = (int) ((midX1 + midX2) / 2 - eX * u2);
				int pY = (int) ((midY1 + midY2) / 2 - eY * u2);
				int qX = (int) ((midX1 + midX2) / 2 - eX * u2 + hX);
				int qY = (int) ((midY1 + midY2) / 2 - eY * u2 + hY);
				int tX = (int) ((midX1 + midX2) / 2 - eX * u2 + fX);
				int tY = (int) ((midY1 + midY2) / 2 - eY * u2 + fY);
				gc.drawText(src.getName(), mX, mY);
				gc.drawText(dest.getName(), nX, nY);
				gc.drawLine(pX, pY, qX, qY);
				gc.drawLine(pX, pY, tX, tY);
			}
		}
		gc.drawLine(aX, aY, bX, bY);
//		gc.drawLine(cX, cY, gX, gY);
//		gc.drawLine(cX, cY, iX, iY);
		RC.gc.setBackground(RC.constants.Colors._Edge_Color);
		
	
		gc.fillPolygon(new int[]{cX,cY,gX,gY,iX,iY});

	}

	public  static void rRenderEdgeD(RenderNode src, RenderNode dest, RenderContext RC)
	{
		Constants constants = RC.constants;
		Transformation T = RC.Trans;
		GC gc = RC.gc;
		int aX = T.tx(src.x());
		int aY = T.ty(src.y());
		int bX = T.tx(dest.x());
		int bY = T.ty(dest.y());

		
		
		
		gc.drawLine(aX, aY, bX, bY);
		
		Point s = dest.size;
		double zX = (aX - bX);
		double zY = (aY - bY);
		double rX = s.x / 2;
		double rY = s.y / 2;
		double dX = zX / rX;
		double dY = zY / rY;
		double L = Math.sqrt(dX * dX + dY * dY);

		// y = 
	
		
		dX /= L;
		dY /= L;
		dX *= rX;
		dY *= rY;
		double L2 = Math.sqrt(dX * dX + dY * dY);
		double dXo = dX / L2;
		double dYo = dY / L2;
		double eX = dXo;
		double eY = dYo;
//		eX /= L2;
//		eY /= L2;
		double rotX = 0.86602540378443864676372317075294;
		double rotY = 0.5;
		int len = constants.ArrowLen;
		double[] t4 = new double[2];
		t4[0] = Math.abs( (dest.size.y/2.0) / dY );
		t4[1] = Math.abs( (dest.size.x/2.0) / dX );
		
		t4[0] = Math.min(t4[0],t4[1]);
		bX = bX + (int)(dX * t4[0]);
		bY = bY + (int)(dY * t4[0]);
		
		

		
		double fX = (rotX * eX - rotY * eY) * len;
		double fY = (rotY * eX + rotX * eY) * len;
		int gX = bX + (int) (0 + fX);
		int gY = bY + (int) (0+ fY);
		double hX = (rotX * eX + rotY * eY) * len;
		double hY = (-rotY * eX + rotX * eY) * len;
		int iX = bX + (int) (0 + hX);
		int iY = bY + (int) (0 + hY);
		int cX = bX ;//+ (int) dX;
		int cY = bY ;//+ (int) dY;
		double midX1 = aX;
		double midY1 = aY;
		double midX2 = bX;
		double midY2 = bY;
		
		
		
		double L4 = Math.sqrt(zX * zX + zY * zY);
		if (L4 >= constants.RoadMapLength)
		{
			if (midX1 < 0)
			{
				midY1 = (aX / dX) * (-dY) + aY;
				midX1 = 0;
			}
			else if (midX1 > T.w())
			{
				midY1 = ((T.w() - bX) / dX) * dY + bY;
				midX1 = T.w();
			}
			if (midX2 < 0)
			{
				midY2 = (bX / dX) * (-dY) + bY;
				midX2 = 0;
			}
			else if (midX2 > T.w())
			{
				midY2 = ((T.w() - aX) / dX) * dY + aY;
				midX2 = T.w();
			}
			if (midY1 < 0)
			{
				midX1 = (aY / dY) * (-dX) + aX;
				midY1 = 0;
			}
			if (midY2 < 0)
			{
				midX2 = (bY / dY) * (-dX) + bX;
				midY2 = 0;
			}
			if (midY1 > T.h())
			{
				midX1 = ((T.h() - bY) / dY) * (dX) + bX;
				midY1 = T.h();
			}
			if (midY2 > T.h())
			{
				midX2 = ((T.h() - aY) / dY) * (dX) + aX;
				midY2 = T.h();
			}
			double kX = midX2 - midX1;
			double kY = midY2 - midY1;
			double L3 = Math.sqrt(kX * kX + kY * kY);
			if (L3 >= 2)
			{
				RC.gc.setBackground(RC.constants.Colors._DefaultBackground);
				
				double u1 = constants.RoadMapRatio * L3;
				double u2 = constants.RoadMapRatioArrow * L3;
				int mX = (int) ((midX1 + midX2) / 2 + eX * u1);
				int mY = (int) ((midY1 + midY2) / 2 + eY * u1);
				int nX = (int) ((midX1 + midX2) / 2 - eX * u1);
				int nY = (int) ((midY1 + midY2) / 2 - eY * u1);
				int pX = (int) ((midX1 + midX2) / 2 - eX * u2);
				int pY = (int) ((midY1 + midY2) / 2 - eY * u2);
				int qX = (int) ((midX1 + midX2) / 2 - eX * u2 + hX);
				int qY = (int) ((midY1 + midY2) / 2 - eY * u2 + hY);
				int tX = (int) ((midX1 + midX2) / 2 - eX * u2 + fX);
				int tY = (int) ((midY1 + midY2) / 2 - eY * u2 + fY);
				gc.drawText(src.getName(), mX, mY);
				gc.drawText(dest.getName(), nX, nY);
				gc.drawLine(pX, pY, qX, qY);
				gc.drawLine(pX, pY, tX, tY);
			}
		}
		gc.drawLine(aX, aY, bX, bY);
//		gc.drawLine(cX, cY, gX, gY);
//		gc.drawLine(cX, cY, iX, iY);
		RC.gc.setBackground(RC.constants.Colors._Edge_Color);
		
	
		gc.fillPolygon(new int[]{cX,cY,gX,gY,iX,iY});
		

	}
	public  static void rRenderEdgeU(RenderNode src, RenderNode dest, RenderContext RC)
	{
		Constants constants = RC.constants;
		Transformation T = RC.Trans;
		GC gc = RC.gc;
		int aX = T.tx(src.x());
		int aY = T.ty(src.y());
		int bX = T.tx(dest.x());
		int bY = T.ty(dest.y());

		
		
		
		gc.drawLine(aX, aY, bX, bY);
		
		Point s = dest.size;
		double zX = (aX - bX);
		double zY = (aY - bY);
		double rX = s.x / 2;
		double rY = s.y / 2;
		double dX = zX / rX;
		double dY = zY / rY;
		double L = Math.sqrt(dX * dX + dY * dY);

		// y = 
	
		
		dX /= L;
		dY /= L;
		dX *= rX;
		dY *= rY;
		double L2 = Math.sqrt(dX * dX + dY * dY);
		double dXo = dX / L2;
		double dYo = dY / L2;
		double eX = dXo;
		double eY = dYo;
//		eX /= L2;
//		eY /= L2;
		double rotX = 0.86602540378443864676372317075294;
		double rotY = 0.5;
		int len = constants.ArrowLen;
		double[] t4 = new double[2];
		
		// y - dest.size.y/2 = (x - dest.size.x/2) * dest.size.y / dest.size.x
		// y = x * dy/dx
		t4[0] = Math.abs( (dest.size.x/2.0) / (1 + dY) );
		//t4[1] = //Math.abs( (dest.size.y/2.0) / (1 + dX) );
		
		double dS = Math.abs(dY / dX);
		//;; y = dest.size.y / 2 - x *dest.size.y / dest.size.x
		
		// y = ds * x
		// y = dest.size/2 - 1/ds * dest.size.y / dest.size.y
		// y = dest.size/2 - x
		// x = t * dest.size.y/dest.size.x
		
		// x = dx * t
		
		// t = (dest.size.y/dest.size.y) / dx
		
		// dy * t - dest.size.y/2 = dx * t
		// -dest.size.y/2 / (dx + dy) = t
		//t4[0] = Math.abs((dest.size.y/dest.size.y) / dX);
		
		t4[0] = Math.abs((dest.size.y/2) / (( dY + dX * dest.size.y / dest.size.x)));
		t4[1] = Math.abs((dest.size.x/2) / (( dX - dY * dest.size.x / dest.size.y)));
		
		/*
		 * 			double mx = Math.abs(lX - size.x/2);
			double my = Math.abs(lY - size.y/2);
			if(size.y/2 - mx * size.y/size.x < my) return false;

		 */
		
		

		// y = dsy/2 - dsy/2 * t
		// x = dsx/2 * t
		
		// y = dy * s
		// x = dx * s
		
		// dy * s = dsy/2 - dsy/2 * t
		// s = dsy/2 - x
		// s = dsy/2 - dx * s
		// s = (dsy/2) / ( 1 + dx)
		
		
		
		
		
		
		// y + dsy/2 = ( x - dsx/2 ) * dsy/dsx
		// y = x*dx/dy
		// x*dx/dy + dsy/2 = x * dsy/dsx - dsy/2
		// x*dx/dy + 
		
		/*
		
		*/
		t4[0] = Math.min(t4[0],t4[1]);
		bX = bX + (int)(dX * t4[0]);
		bY = bY + (int)(dY * t4[0]);
		
		
		double fX = (rotX * eX - rotY * eY) * len;
		double fY = (rotY * eX + rotX * eY) * len;
		int gX = bX + (int) (0 + fX);
		int gY = bY + (int) (0+ fY);
		double hX = (rotX * eX + rotY * eY) * len;
		double hY = (-rotY * eX + rotX * eY) * len;
		int iX = bX + (int) (0 + hX);
		int iY = bY + (int) (0 + hY);
		int cX = bX ;//+ (int) dX;
		int cY = bY ;//+ (int) dY;
		double midX1 = aX;
		double midY1 = aY;
		double midX2 = bX;
		double midY2 = bY;
		
		
		
		double L4 = Math.sqrt(zX * zX + zY * zY);
		if (L4 >= constants.RoadMapLength)
		{
			if (midX1 < 0)
			{
				midY1 = (aX / dX) * (-dY) + aY;
				midX1 = 0;
			}
			else if (midX1 > T.w())
			{
				midY1 = ((T.w() - bX) / dX) * dY + bY;
				midX1 = T.w();
			}
			if (midX2 < 0)
			{
				midY2 = (bX / dX) * (-dY) + bY;
				midX2 = 0;
			}
			else if (midX2 > T.w())
			{
				midY2 = ((T.w() - aX) / dX) * dY + aY;
				midX2 = T.w();
			}
			if (midY1 < 0)
			{
				midX1 = (aY / dY) * (-dX) + aX;
				midY1 = 0;
			}
			if (midY2 < 0)
			{
				midX2 = (bY / dY) * (-dX) + bX;
				midY2 = 0;
			}
			if (midY1 > T.h())
			{
				midX1 = ((T.h() - bY) / dY) * (dX) + bX;
				midY1 = T.h();
			}
			if (midY2 > T.h())
			{
				midX2 = ((T.h() - aY) / dY) * (dX) + aX;
				midY2 = T.h();
			}
			double kX = midX2 - midX1;
			double kY = midY2 - midY1;
			double L3 = Math.sqrt(kX * kX + kY * kY);
			if (L3 >= 2)
			{
				RC.gc.setBackground(RC.constants.Colors._DefaultBackground);
				
				double u1 = constants.RoadMapRatio * L3;
				double u2 = constants.RoadMapRatioArrow * L3;
				int mX = (int) ((midX1 + midX2) / 2 + eX * u1);
				int mY = (int) ((midY1 + midY2) / 2 + eY * u1);
				int nX = (int) ((midX1 + midX2) / 2 - eX * u1);
				int nY = (int) ((midY1 + midY2) / 2 - eY * u1);
				int pX = (int) ((midX1 + midX2) / 2 - eX * u2);
				int pY = (int) ((midY1 + midY2) / 2 - eY * u2);
				int qX = (int) ((midX1 + midX2) / 2 - eX * u2 + hX);
				int qY = (int) ((midY1 + midY2) / 2 - eY * u2 + hY);
				int tX = (int) ((midX1 + midX2) / 2 - eX * u2 + fX);
				int tY = (int) ((midY1 + midY2) / 2 - eY * u2 + fY);
				gc.drawText(src.getName(), mX, mY);
				gc.drawText(dest.getName(), nX, nY);
				gc.drawLine(pX, pY, qX, qY);
				gc.drawLine(pX, pY, tX, tY);
			}
		}
		gc.drawLine(aX, aY, bX, bY);
//		gc.drawLine(cX, cY, gX, gY);
//		gc.drawLine(cX, cY, iX, iY);
		RC.gc.setBackground(RC.constants.Colors._Edge_Color);
		
	
		gc.fillPolygon(new int[]{cX,cY,gX,gY,iX,iY});

	}
	
	
	public String getName()
	{
		return "Edge";
	}
}