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
 */package edu.ksu.cis.bnj.gui.tools;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Font;
import org.eclipse.swt.widgets.Display;
import edu.ksu.cis.util.GlobalOptions;
public class FontWheel
{
	public Font	CPF_Body;
	public Font	Node_Body;
	public Font	RoadMap;
	public Font NodeOrder;
	public Font VisMarking;
	public Font CodePageActive;
	public Font CodePageInactive;
	public Font VisHeader;
	public Font ColorLegend;
	
	public void init(Display disp)
	{
		GlobalOptions GO = GlobalOptions.getInstance();
		//CPF_Header = new Font(disp, "Arial", 12, SWT.NORMAL);
		CPF_Body = new Font(disp, GO.getString("font_cpf", "Verdana"), GO.getInteger("fontsize_cpf", 9), SWT.NORMAL);
		Node_Body = new Font(disp, GO.getString("font_node", "Verdana"), GO.getInteger("fontsize_node", 10), SWT.NORMAL);
		RoadMap = new Font(disp, GO.getString("font_roadmap", "Verdana"), GO.getInteger("fontsize_roadmap", 8),	SWT.NORMAL);
		
		NodeOrder = new Font(disp, GO.getString("font_nodeorder", "Times New Roman Unicode"), GO.getInteger("fontsize_nodeorder", 8),	SWT.NORMAL);
		VisMarking = new Font(disp, GO.getString("font_marking", "Times New Roman Unicode"), GO.getInteger("fontsize_marking", 8),	SWT.NORMAL);
		CodePageActive = new Font(disp, GO.getString("font_codepage_active", "Times New Roman Unicode"), GO.getInteger("fontsize_codepage_active", 8),	SWT.NORMAL);
		CodePageInactive = new Font(disp, GO.getString("font_codepage_inactive", "Times New Roman Unicode"), GO.getInteger("fontsize_codepage_inactive", 8),	SWT.NORMAL);
		VisHeader = new Font(disp, GO.getString("font_vis_header", "Times New Roman Unicode"), GO.getInteger("fontsize_vis_header", 16),	SWT.NORMAL);
		
		ColorLegend = new Font(disp, GO.getString("font_vis_header", "Times New Roman Unicode"), GO.getInteger("fontsize_vis_colorlegend", 10),	SWT.NORMAL);
	}
	public void clean()
	{
		ColorLegend.dispose();
		CPF_Body.dispose();
		Node_Body.dispose();
		RoadMap.dispose();
		NodeOrder.dispose();
		VisMarking.dispose();
		CodePageActive.dispose();
		CodePageInactive.dispose();
		VisHeader.dispose();
	}
}