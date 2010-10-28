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
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.widgets.Display;
import edu.ksu.cis.util.GlobalOptions;
import edu.ksu.cis.util.data.IntTriplet;
public class ColorWheel
{
	public Color	_DefaultText				= null;
	public Color	_DefaultBackground			= null;
	public Color	_CPF_Background_Header		= null;
	public Color	_CPF_Text_Header			= null;
	public Color	_CPF_Text					= null;
	public Color	_Desc_Color					= null;
	public Color	_Util_Color					= null;
	public Color	_Node_Color					= null;
	public Color	_Edge_Color					= null;
	public Color	_Node_Background			= null;
	public Color	_Node_Border				= null;
	public Color	_Node_Background_Selected	= null;
	public Color	_Node_Border_Selected		= null;
	public Color	_Cursor						= null;
	public Color	_TranslateVector			= null;
	public Color	_EnableCPF_Back				= null;
	public Color	_EnableCPF_Border			= null;
	public Color	_EnableCPF_Text				= null;
	public Color	_GridColor					= null;
	public Color	_NodeEditColorGradStart		= null;
	public Color	_NodeEditColorGradEnd		= null;
	public Color	_InvalidCPFValue			= null;
	public Color[]	_NodeBackColor				= null;
	public Color[]	_NodeForeColor				= null;
	public Color	_VisOrderBox				= null;
	public Color	_VisOrderBoxFore			= null;
	public Color	_VisOrderBoxBorder			= null;
	public Color	_VisMarkingColor			= null;
	
	public Color	_VisCodePageBackActive		= null;
	public Color	_VisCodePageBackInactive	= null;
	public Color	_VisCodePageForeActive		= null;
	public Color	_VisCodePageForeInactive	= null;
	public Color	_VisCodePageForeBoxActive	= null;
	public Color	_VisHeader					= null;
	public Color	_Edge_Color_Selected		= null;
	public Color	_EdgeTemp_Color					= null;
	
	public Color 	_ColorLegendBorder			= null;
	
	public Color newColor(Display disp, int defx, int defy, int defz, String name)
	{
		GlobalOptions GO = GlobalOptions.getInstance();
		IntTriplet RGB = GO.getIntTriplet(name, defx, defy, defz);
		Color C = new Color(disp, RGB.x, RGB.y, RGB.z);
		return C;
	}
	ColorWheel()
	{
		_NodeBackColor = new Color[36];
		_NodeForeColor = new Color[36];
	}
	public void init(Display disp)
	{
		_ColorLegendBorder = newColor(disp, 0, 0, 0, "color_legend_border"); 
		_DefaultText = newColor(disp, 0, 0, 0, "color_default_text");
		_DefaultBackground = newColor(disp, 255, 255, 255, "color_default_background");
		_CPF_Text_Header = newColor(disp, 0, 0, 0, "color_cpf_header");
		_CPF_Background_Header = newColor(disp, 240,230,140, "color_cpf_background_header");
	
		_EdgeTemp_Color = newColor(disp, 178, 34, 34, "color_edgetemp");
		
		//107 77 156
		// default node (0) colors
		_Node_Color = newColor(disp, 0, 0,0, "color_node");
		_Node_Background = newColor(disp, 31,221,255 , "color_node_background");
		_NodeForeColor[0]  = _Node_Color;
		_NodeBackColor[0]  = _Node_Background;

		//moralization colors
		_NodeBackColor[1]  = newColor(disp, 220, 20, 60, "color_node_back1");
		_NodeBackColor[2]  = newColor(disp, 255, 239, 213, "color_node_back2");
		_NodeBackColor[3]  = newColor(disp, 204, 127, 50, "color_node_back3");
		// remove directionality
		_NodeBackColor[4]  = newColor(disp, 66, 211, 66, "color_node_back4");
		// MCS77, 77,255  255, 99, 71 
		_NodeBackColor[5]  = newColor(disp, 77, 77, 255, "color_node_back5");
		_NodeBackColor[6]  = newColor(disp, 255, 99, 71, "color_node_back6");
		// FillIn
		_NodeBackColor[7]  = newColor(disp, 255, 239, 213, "color_node_back7");
		_NodeBackColor[8]  = newColor(disp, 204, 127, 50, "color_node_back8");
		_NodeBackColor[9]  = newColor(disp, 65, 105, 225, "color_node_back9");
		// build clique tree
		_NodeBackColor[10] = newColor(disp, 184, 115, 51, "color_node_back10");
		_NodeBackColor[11] = newColor(disp, 65, 105, 225, "color_node_back11");
		_NodeBackColor[12] = newColor(disp, 66, 211, 66, "color_node_back12");
		_NodeBackColor[13] = newColor(disp, 255, 239, 213, "color_node_back13");
		// connecting
		_NodeBackColor[20] = newColor(disp, 238, 232, 170, "color_node_back20");
		_NodeBackColor[21] = newColor(disp, 0, 128, 128, "color_node_back21");
		_NodeBackColor[22] = newColor(disp, 135, 206, 235, "color_node_back22");
		// ls messages
		_NodeBackColor[14] = newColor(disp, 0, 191, 255, "color_node_back14");
		_NodeBackColor[15] = newColor(disp, 124, 252, 0, "color_node_back15");
		_NodeBackColor[16] = newColor(disp, 240, 230, 140, "color_node_back16");
		_NodeBackColor[17] = newColor(disp, 228, 120, 51, "color_node_back17");
		// wrap up
		_NodeBackColor[18] = newColor(disp, 65, 105, 225, "color_node_back18");
		_NodeBackColor[19] = newColor(disp, 107, 77, 156, "color_node_back19");
		_NodeForeColor[19] = newColor(disp, 255, 255, 255, "color_node_fore19");

		// intout colors
		_NodeBackColor[23] = newColor(disp, 107, 77, 156, "color_node_back23");
		_NodeBackColor[24] = newColor(disp, 65, 105, 225, "color_node_back24");
		_NodeForeColor[23] = newColor(disp, 255, 255, 255, "color_node_fore23");
		_NodeForeColor[24] = newColor(disp, 255, 255, 255, "color_node_fore24");

		_Desc_Color = newColor(disp, 113, 247, 141, "color_node_desc");
		_Util_Color = newColor(disp, 252, 148, 102, "color_node_util");

		
		for(int i = 25; i < 36; i++)
		{
			if(_NodeBackColor[i]!=null)
				_NodeBackColor[i].dispose();
			_NodeBackColor[i] = newColor(disp, 100, 100, 100, "color_node_back"+i);
		}
		for(int i = 1; i < 36; i++)
		{
			if(_NodeForeColor[i]!=null)
				_NodeForeColor[i].dispose();
			_NodeForeColor[i]= newColor(disp, 0, 0, 0, "color_node_fore"+i);
		}		
		
		_VisOrderBox  = newColor(disp, 65,105,225, "color_vis_node_order_box");
		_VisOrderBoxFore = newColor(disp, 255, 255, 255, "color_vis_node_order_box_foreground");
		_VisOrderBoxBorder = newColor(disp, 0, 0, 0, "color_vis_node_order_box_fore_border");
		_VisMarkingColor = newColor(disp, 0, 0, 0, "color_vis_marking");
		_Node_Border = newColor(disp,  25, 25,112, "color_node_border");
		_Edge_Color = newColor(disp, 0, 0, 0, "color_edge");
		_Node_Background_Selected = newColor(disp, 255,145,148, "color_node_background_selected");
		_Node_Border_Selected = newColor(disp, 255,  0,  0, "color_node_border_selected");
		
		_Cursor = newColor(disp, 30,144,255, "color_cursor");
		_TranslateVector = newColor(disp, 107, 66, 38, "color_translate_vector");
		
		_EnableCPF_Back = newColor(disp, 106, 90,205, "color_enable_cpf_back");
		_EnableCPF_Border = newColor(disp, 33, 94, 33, "color_enable_cpf_border");
		_EnableCPF_Text = newColor(disp,  255,  255,139, "color_enable_cpf_text");
		
		_GridColor = newColor(disp, 220,220,220, "color_grid_color");
		_NodeEditColorGradStart = newColor(disp, 255, 255, 255, "color_node_edit_gradient_start");
		_NodeEditColorGradEnd = newColor(disp, 107, 77, 156 , "color_node_edit_gradient_end");
		
		_InvalidCPFValue = newColor(disp, 255, 0, 0, "color_invalid_cpf_value");
		_CPF_Text = newColor(disp, 0, 0, 0, "color_cpf_text");
		_Edge_Color_Selected = newColor(disp, 70,130,180, "color_edge_selected");
		
		_VisCodePageBackActive		= newColor(disp, 233,194,166, "color_vis_codepage_back_active");
		_VisCodePageBackInactive	= newColor(disp, 255, 255, 255, "color_vis_codepage_back_inactive");
		
		
		_VisCodePageForeActive		= newColor(disp, 0, 0, 0, "color_vis_codepage_fore_active");
		_VisCodePageForeInactive	= newColor(disp, 112,128,144, "color_vis_codepage_fore_inactive");
		_VisCodePageForeBoxActive	= newColor(disp, 178, 34, 34, "color_vis_codepage_box_active");
		_VisHeader = newColor(disp, 72, 61,139, "color_vis_header");
	}
	public void clean()
	{
		_Desc_Color.dispose();
		_Util_Color.dispose();
		
		_EdgeTemp_Color.dispose();
		_DefaultText.dispose();
		_DefaultBackground.dispose();
		_CPF_Text_Header.dispose();
		_CPF_Background_Header.dispose();
		_Node_Color.dispose();
		_Node_Background.dispose();
		_Node_Border.dispose();
		_Edge_Color.dispose();
		_Node_Background_Selected.dispose();
		_Node_Border_Selected.dispose();
		_Cursor.dispose();
		_TranslateVector.dispose();
		_EnableCPF_Back.dispose();
		_EnableCPF_Border.dispose();
		_EnableCPF_Text.dispose();
		_GridColor.dispose();
		_NodeEditColorGradStart.dispose();
		_NodeEditColorGradEnd.dispose();
		_InvalidCPFValue.dispose();
		_CPF_Text.dispose();
		_VisOrderBox.dispose();
		for(int i = 1; i < 36; i++)
		{
			_NodeBackColor[i].dispose();
			_NodeForeColor[i].dispose();
		}
		_VisOrderBoxFore.dispose();
		_VisOrderBoxBorder.dispose();
		_VisMarkingColor.dispose();
		_VisCodePageBackActive.dispose();
		_VisCodePageBackInactive.dispose();
		_VisCodePageForeActive.dispose();
		_VisCodePageForeInactive.dispose();
		_VisCodePageForeBoxActive.dispose();
		_VisHeader.dispose();
		_ColorLegendBorder.dispose();
	}
}