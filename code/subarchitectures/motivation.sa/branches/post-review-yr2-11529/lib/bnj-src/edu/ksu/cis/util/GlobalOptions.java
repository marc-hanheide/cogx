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
 */package edu.ksu.cis.util;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Iterator;
import java.util.TreeMap;
import edu.ksu.cis.util.data.IntTriplet;
/**
 * file: GlobalOptions.java
 * 
 * @author Jeffrey M. Barber
 */
public class GlobalOptions
{
	private TreeMap	_mapping;
	public void set(String key, Object val)
	{	_mapping.put(key, val);		}
	public Object get(String name)
	{	return _mapping.get(name);	}
	static GlobalOptions	instance	= new GlobalOptions();
	public boolean			debugmode	= true;
	static public GlobalOptions getInstance()
	{	return instance;			}
	
	
	TreeMap _colorname2html;
	TreeMap _html2colorname;
	public void BuildColor()
	{
		_colorname2html = new TreeMap();
		_html2colorname = new TreeMap();
		IntTriplet._html2colorname = _html2colorname;
		_colorname2html.put("MAROON","#800000");
		_colorname2html.put("PURPLE","#800080");
		_colorname2html.put("DARKMAGENTA","#8B008B");
		_colorname2html.put("DARKPURPLE","#871F78");
		_colorname2html.put("MAROONII","#8E236B");
		_colorname2html.put("INDIGO","#4B0082");
		_colorname2html.put("BLUEVIOLET","#8A2BE2");
		_colorname2html.put("DARKVIOLET","#9400D3");
		_colorname2html.put("VIOLETRED","#CC3299");
		_colorname2html.put("MEDIUMVIOLETRED","#C71585");
		_colorname2html.put("MEDIUMPURPLE","#9370DB");
		_colorname2html.put("MAGENTA","#FF00FF");
		_colorname2html.put("DARKORCHID","#9932CC");
		_colorname2html.put("MEDIUMORCHID","#BA55D3");
		_colorname2html.put("PALEVIOLETRED","#DB7093");
		_colorname2html.put("ORCHID","#DA70D6");
		_colorname2html.put("VIOLET","#EE82EE");
		_colorname2html.put("PLUMII","#EAADEA");
		_colorname2html.put("PLUM","#DDA0DD");
		_colorname2html.put("THISTLE","#D8BFD8");
		_colorname2html.put("LAVENDERBLUSH","#FFF0F5");
		_colorname2html.put("SNOW","#FFFAFA");
		_colorname2html.put("WHITESMOKE","#F5F5F5");
		_colorname2html.put("LAVENDER","#E6E6FA");
		_colorname2html.put("HONEYDEW","#F0FFF0");
		_colorname2html.put("MINTCREAM","#F5FFFA");
		_colorname2html.put("ALICEBLUE","#F0F8FF");
		_colorname2html.put("AZURE","#F0FFFF");
		_colorname2html.put("QUARTZ","#D9D9F3");
		_colorname2html.put("LIGHTCYAN","#E0FFFF");
		_colorname2html.put("LIGHTBLUEII","#BFD8D8");
		_colorname2html.put("LIGHTSTEELBLUE","#B0C4DE");
		_colorname2html.put("POWDERBLUE","#B0E0E6");
		_colorname2html.put("PALETURQUOISE","#AFEEEE");
		_colorname2html.put("LIGHTBLUE","#ADD8E6");
		_colorname2html.put("SKYBLUE","#87CEEB");
		_colorname2html.put("LIGHTSKYBLUE","#87CEFA");
		_colorname2html.put("DARKTURQUOISEII","#7093DB");
		_colorname2html.put("LIGHTSLATEBLUE","#8470FF");
		_colorname2html.put("CORNFLOWERBLUE","#6495ED");
		_colorname2html.put("SLATEBLUE","#6A5ACD");
		_colorname2html.put("MEDIUMSLATEBLUE","#7B68EE");
		_colorname2html.put("LIGHTSTEELBLUEII","#8F8FBD");
		_colorname2html.put("CADETBLUE","#5F9EA0");
		_colorname2html.put("RICHBLUE","#5959AB");
		_colorname2html.put("NEONBLUE","#4D4DFF");
		_colorname2html.put("BLUE","#0000FF");
		_colorname2html.put("MEDIUMBLUEII","#3232CD");
		_colorname2html.put("MEDIUMBLUE","#0000CD");
		_colorname2html.put("NEWMIDNIGHTBLUE","#00009C");
		_colorname2html.put("VERYDARKBLUE","#333399");
		_colorname2html.put("NAVYBLUE","#23238E");
		_colorname2html.put("NAVY","#000080");
		_colorname2html.put("DARKBLUE","#00008B");
		_colorname2html.put("MIDNIGHTBLUE","#191970");
		_colorname2html.put("MIDNIGHTBLUEII","#2F2F4F");
		_colorname2html.put("DARKSLATEBLUE","#483D8B");
		_colorname2html.put("TEAL","#008080");
		_colorname2html.put("STEELBLUEII","#236B8E");
		_colorname2html.put("DARKCYAN","#008B8B");
		_colorname2html.put("DEEPSKYBLUEII","#00BFBF");
		_colorname2html.put("DODGERBLUE","#1E90FF");
		_colorname2html.put("ROYALBLUE","#4169E1");
		_colorname2html.put("DARKTURQUOISE","#00CED1");
		_colorname2html.put("DEEPSKYBLUE","#00BFFF");
		_colorname2html.put("AQUA(CYAN)","#00FFFF");
		_colorname2html.put("SEAGREENII","#238E6B");
		_colorname2html.put("SEAGREEN","#2E8B57");
		_colorname2html.put("MEDIUMSEAGREEN","#3CB371");
		_colorname2html.put("MEDIUMSPRINGGREEN","#00FA9A");
		_colorname2html.put("SPRINGGREEN","#00FF7F");
		_colorname2html.put("TURQUOISE","#40E0D0");
		_colorname2html.put("MEDIUMAQUAMARINEII","#32CC99");
		_colorname2html.put("LIGHTSEAGREEN","#20B2AA");
		_colorname2html.put("SKYBLUEII","#3299CC");
		_colorname2html.put("SUMMERSKY","#38B0DE");
		_colorname2html.put("STEELBLUE","#4682B4");
		_colorname2html.put("MEDIUMTURQUOISE","#48D1CC");
		_colorname2html.put("AQUAMARINE","#7FFFD4");
		_colorname2html.put("MEDIUMTURQUOISEII","#70DBDB");
		_colorname2html.put("MEDIUMAQUAMARINE","#66CDAA");
		_colorname2html.put("AQUAMARINEII","#70DB93");
		_colorname2html.put("DARKSEAGREEN","#8FBC8F");
		_colorname2html.put("PALEGREEN","#98FB98");
		_colorname2html.put("LIGHTGREEN","#90EE90");
		_colorname2html.put("LIMEGREEN","#32CD32");
		_colorname2html.put("MEDIUMSEAGREENII","#42D342");
		_colorname2html.put("GREEN","#00FF00");
		_colorname2html.put("FORESTGREEN","#228B22");
		_colorname2html.put("GREENII","#008000");
		_colorname2html.put("BRASS","#B5A642");
		_colorname2html.put("DARKGOLDENROD","#B8860B");
		_colorname2html.put("COPPER","#B87333");
		_colorname2html.put("GOLDII","#CC7F32");
		_colorname2html.put("GOLDENROD","#DAA520");
		_colorname2html.put("OLDGOLD","#CFB53B");
		_colorname2html.put("BRIGHTGOLD","#D9D919");
		_colorname2html.put("GOLD","#FFD700");
		_colorname2html.put("YELLOW","#FFFF00");
		_colorname2html.put("GOLDENRODII","#DBDB70");
		_colorname2html.put("KHAKI","#F0E68C");
		_colorname2html.put("MEDIUMGOLDENROD","#EAEAAE");
		_colorname2html.put("PALEGOLDENROD","#EEE8AA");
		_colorname2html.put("LEMONCHIFFON","#FFFACD");
		_colorname2html.put("LIGHTGOLDENRODYELLOW","#FAFAD2");
		_colorname2html.put("LIGHTYELLOW","#FFFFE0");
		_colorname2html.put("BEIGE","#F5F5DC");
		_colorname2html.put("CORNSILK","#FFF8DC");
		_colorname2html.put("IVORY","#FFFFF0");
		_colorname2html.put("PAPAYAWHIP","#FFEFD5");
		_colorname2html.put("OLDLACE","#FDF5E6");
		_colorname2html.put("BISQUE","#FFE4C4");
		_colorname2html.put("BLANCHEDALMOND","#FFEBCD");
		_colorname2html.put("ANTIQUEWHITE","#FAEBD7");
		_colorname2html.put("FLORALWHITE","#FFFAF0");
		_colorname2html.put("LINEN","#FAF0E6");
		_colorname2html.put("WHEAT","#F5DEB3");
		_colorname2html.put("WHEATII","#D8D8BF");
		_colorname2html.put("MOCCASIN","#FFE4B5");
		_colorname2html.put("NAVAJOWHITE","#FFDEAD");
		_colorname2html.put("NEWTAN","#EBC79E");
		_colorname2html.put("LIGHTWOOD","#E9C2A6");
		_colorname2html.put("BURLYWOOD","#DEB887");
		_colorname2html.put("TAN","#D2B48C");
		_colorname2html.put("FELDSPAR","#D19275");
		_colorname2html.put("PERU","#CD853F");
		_colorname2html.put("MEDIUMWOOD","#A68064");
		_colorname2html.put("DARKTAN","#97694F");
		_colorname2html.put("DARKWOOD","#855E42");
		_colorname2html.put("SADDLEBROWN","#8B4513");
		_colorname2html.put("SIENNAII","#8E6B23");
		_colorname2html.put("SIENNA","#A0522D");
		_colorname2html.put("BROWN","#A52A2A");
		_colorname2html.put("CHOCOLATE","#D2691E");
		_colorname2html.put("SEMI-SWEETCHOCOLATE","#6B4226");
		_colorname2html.put("DARKBROWN","#5C4033");
		_colorname2html.put("BAKER'SCHOCOLATE","#5C3317");
		_colorname2html.put("DUSTYROSE","#856363");
		_colorname2html.put("ROSYBROWN","#BC8F8F");
		_colorname2html.put("SANDYBROWN","#F4A460");
		_colorname2html.put("MANDARIANORANGE","#E47833");
		_colorname2html.put("COOLCOPPER","#D98719");
		_colorname2html.put("ORANGE","#FFA500");
		_colorname2html.put("DARKORANGE","#FF8C00");
		_colorname2html.put("ORANGEII","#FF7F00");
		_colorname2html.put("ORANGERED","#FF4500");
		_colorname2html.put("TOMATO","#FF6347");
		_colorname2html.put("ORANGEREDII","#FF2400");
		_colorname2html.put("RED","#FF0000");
		_colorname2html.put("DARKRED","#8B0000");
		_colorname2html.put("SCARLET","#8C1717");
		_colorname2html.put("FIREBRICKII","#8E2323");
		_colorname2html.put("FIREBRICK","#B22222");
		_colorname2html.put("CRIMSON","#DC143C");
		_colorname2html.put("INDIANRED","#CD5C5C");
		_colorname2html.put("LIGHTCORAL","#F08080");
		_colorname2html.put("SALMON","#FA8072");
		_colorname2html.put("DARKSALMON","#E9967A");
		_colorname2html.put("CORAL","#FF7F50");
		_colorname2html.put("LIGHTSALMON","#FFA07A");
		_colorname2html.put("SEASHELL","#FFF5EE");
		_colorname2html.put("MISTYROSE","#FFE4E1");
		_colorname2html.put("PINK","#FFC0CB");
		_colorname2html.put("LIGHTPINK","#FFB6C1");
		_colorname2html.put("NEONPINK","#FF6EC7");
		_colorname2html.put("HOTPINK","#FF69B4");
		_colorname2html.put("SPICYPINK","#FF1CAE");
		_colorname2html.put("DEEPPINK","#FF1493");
		_colorname2html.put("BLACK","#000000");
		_colorname2html.put("DARKSLATEGRAY","#2F4F4F");
		_colorname2html.put("DIMGRAY","#696969");
		_colorname2html.put("DARKGRAY","#808080");
		_colorname2html.put("SLATEGRAY","#708090");
		_colorname2html.put("LIGHTSLATEGRAY","#778899");
		_colorname2html.put("GRAY","#A9A9A9");
		_colorname2html.put("SILVER","#C0C0C0");
		_colorname2html.put("LIGHTGRAY","#CDCDCD");
		_colorname2html.put("VERYLIGHTGRAY","#D3D3D3");
		_colorname2html.put("GAINSBORO","#DCDCDC");
		_colorname2html.put("GHOSTWHITE","#F8F8FF");
		_colorname2html.put("WHITE","#FFFFFF");
		_html2colorname.put("#800000","Maroon");
		_html2colorname.put("#800080","Purple");
		_html2colorname.put("#8B008B","Dark Magenta");
		_html2colorname.put("#871F78","Dark Purple");
		_html2colorname.put("#8E236B","Maroon II");
		_html2colorname.put("#4B0082","Indigo");
		_html2colorname.put("#8A2BE2","Blue Violet");
		_html2colorname.put("#9400D3","Dark Violet");
		_html2colorname.put("#CC3299","Violet Red");
		_html2colorname.put("#C71585","Medium Violet Red");
		_html2colorname.put("#9370DB","Medium Purple");
		_html2colorname.put("#FF00FF","Magenta");
		_html2colorname.put("#9932CC","Dark Orchid");
		_html2colorname.put("#BA55D3","Medium Orchid");
		_html2colorname.put("#DB7093","Pale Violet Red");
		_html2colorname.put("#DA70D6","Orchid");
		_html2colorname.put("#EE82EE","Violet");
		_html2colorname.put("#EAADEA","Plum II");
		_html2colorname.put("#DDA0DD","Plum");
		_html2colorname.put("#D8BFD8","Thistle");
		_html2colorname.put("#FFF0F5","Lavender Blush");
		_html2colorname.put("#FFFAFA","Snow");
		_html2colorname.put("#F5F5F5","White Smoke");
		_html2colorname.put("#E6E6FA","Lavender");
		_html2colorname.put("#F0FFF0","Honeydew");
		_html2colorname.put("#F5FFFA","Mint Cream");
		_html2colorname.put("#F0F8FF","Alice Blue");
		_html2colorname.put("#F0FFFF","Azure");
		_html2colorname.put("#D9D9F3","Quartz");
		_html2colorname.put("#E0FFFF","Light Cyan");
		_html2colorname.put("#BFD8D8","Light Blue II");
		_html2colorname.put("#B0C4DE","Light Steel Blue");
		_html2colorname.put("#B0E0E6","Powder Blue");
		_html2colorname.put("#AFEEEE","Pale Turquoise");
		_html2colorname.put("#ADD8E6","Light Blue");
		_html2colorname.put("#87CEEB","Sky Blue");
		_html2colorname.put("#87CEFA","Light Sky Blue");
		_html2colorname.put("#7093DB","Dark Turquoise II");
		_html2colorname.put("#8470FF","Light Slate Blue");
		_html2colorname.put("#6495ED","Cornflower Blue");
		_html2colorname.put("#6A5ACD","Slate Blue");
		_html2colorname.put("#7B68EE","Medium Slate Blue");
		_html2colorname.put("#8F8FBD","Light Steel Blue II");
		_html2colorname.put("#5F9EA0","Cadet Blue");
		_html2colorname.put("#5959AB","Rich Blue");
		_html2colorname.put("#4D4DFF","Neon Blue");
		_html2colorname.put("#0000FF","Blue");
		_html2colorname.put("#3232CD","Medium Blue II");
		_html2colorname.put("#0000CD","Medium Blue");
		_html2colorname.put("#00009C","New Midnight Blue");
		_html2colorname.put("#333399","Very Dark Blue");
		_html2colorname.put("#23238E","Navy Blue");
		_html2colorname.put("#000080","Navy");
		_html2colorname.put("#00008B","Dark Blue");
		_html2colorname.put("#191970","Midnight Blue");
		_html2colorname.put("#2F2F4F","Midnight Blue II");
		_html2colorname.put("#483D8B","Dark Slate Blue");
		_html2colorname.put("#008080","Teal");
		_html2colorname.put("#236B8E","Steel Blue II");
		_html2colorname.put("#008B8B","Dark Cyan");
		_html2colorname.put("#00BFBF","Deep Sky Blue II");
		_html2colorname.put("#1E90FF","Dodger Blue");
		_html2colorname.put("#4169E1","Royal Blue");
		_html2colorname.put("#00CED1","Dark Turquoise");
		_html2colorname.put("#00BFFF","Deep Sky Blue");
		_html2colorname.put("#00FFFF","Aqua (Cyan)");
		_html2colorname.put("#238E6B","Sea Green II");
		_html2colorname.put("#2E8B57","Sea Green");
		_html2colorname.put("#3CB371","Medium Sea Green");
		_html2colorname.put("#00FA9A","Medium Spring Green");
		_html2colorname.put("#00FF7F","Spring Green");
		_html2colorname.put("#40E0D0","Turquoise");
		_html2colorname.put("#32CC99","Medium Aquamarine II");
		_html2colorname.put("#20B2AA","Light Sea Green");
		_html2colorname.put("#3299CC","Sky Blue II");
		_html2colorname.put("#38B0DE","Summer Sky");
		_html2colorname.put("#4682B4","Steel Blue");
		_html2colorname.put("#48D1CC","Medium Turquoise");
		_html2colorname.put("#7FFFD4","Aquamarine");
		_html2colorname.put("#70DBDB","Medium Turquoise II");
		_html2colorname.put("#66CDAA","Medium Aquamarine");
		_html2colorname.put("#70DB93","Aquamarine II");
		_html2colorname.put("#8FBC8F","Dark Sea Green");
		_html2colorname.put("#98FB98","Pale Green");
		_html2colorname.put("#90EE90","Light Green");
		_html2colorname.put("#32CD32","Lime Green");
		_html2colorname.put("#42D342","Medium Sea Green II");
		_html2colorname.put("#00FF00","Green");
		_html2colorname.put("#228B22","Forest Green");
		_html2colorname.put("#008000","Green II");
		_html2colorname.put("#B5A642","Brass");
		_html2colorname.put("#B8860B","Dark Goldenrod");
		_html2colorname.put("#B87333","Copper");
		_html2colorname.put("#CC7F32","Gold II");
		_html2colorname.put("#DAA520","Goldenrod");
		_html2colorname.put("#CFB53B","Old Gold");
		_html2colorname.put("#D9D919","Bright Gold");
		_html2colorname.put("#FFD700","Gold");
		_html2colorname.put("#FFFF00","Yellow");
		_html2colorname.put("#DBDB70","Goldenrod II");
		_html2colorname.put("#F0E68C","Khaki");
		_html2colorname.put("#EAEAAE","Medium Goldenrod");
		_html2colorname.put("#EEE8AA","Pale Goldenrod");
		_html2colorname.put("#FFFACD","Lemon Chiffon");
		_html2colorname.put("#FAFAD2","Light Goldenrod Yellow");
		_html2colorname.put("#FFFFE0","Light Yellow");
		_html2colorname.put("#F5F5DC","Beige");
		_html2colorname.put("#FFF8DC","Cornsilk");
		_html2colorname.put("#FFFFF0","Ivory");
		_html2colorname.put("#FFEFD5","Papaya Whip");
		_html2colorname.put("#FDF5E6","Old Lace");
		_html2colorname.put("#FFE4C4","Bisque");
		_html2colorname.put("#FFEBCD","Blanched Almond");
		_html2colorname.put("#FAEBD7","Antique White");
		_html2colorname.put("#FFFAF0","Floral White");
		_html2colorname.put("#FAF0E6","Linen");
		_html2colorname.put("#F5DEB3","Wheat");
		_html2colorname.put("#D8D8BF","Wheat II");
		_html2colorname.put("#FFE4B5","Moccasin");
		_html2colorname.put("#FFDEAD","Navajo White");
		_html2colorname.put("#EBC79E","New Tan");
		_html2colorname.put("#E9C2A6","Light Wood");
		_html2colorname.put("#DEB887","Burly Wood");
		_html2colorname.put("#D2B48C","Tan");
		_html2colorname.put("#D19275","Feldspar");
		_html2colorname.put("#CD853F","Peru");
		_html2colorname.put("#A68064","Medium Wood");
		_html2colorname.put("#97694F","Dark Tan");
		_html2colorname.put("#855E42","Dark Wood");
		_html2colorname.put("#8B4513","Saddle Brown");
		_html2colorname.put("#8E6B23","Sienna II");
		_html2colorname.put("#A0522D","Sienna");
		_html2colorname.put("#A52A2A","Brown");
		_html2colorname.put("#D2691E","Chocolate");
		_html2colorname.put("#6B4226","Semi-Sweet Chocolate");
		_html2colorname.put("#5C4033","Dark Brown");
		_html2colorname.put("#5C3317","Baker's Chocolate");
		_html2colorname.put("#856363","Dusty Rose");
		_html2colorname.put("#BC8F8F","Rosy Brown");
		_html2colorname.put("#F4A460","Sandy Brown");
		_html2colorname.put("#E47833","Mandarian Orange");
		_html2colorname.put("#D98719","Cool Copper");
		_html2colorname.put("#FFA500","Orange");
		_html2colorname.put("#FF8C00","Dark Orange");
		_html2colorname.put("#FF7F00","Orange II");
		_html2colorname.put("#FF4500","Orange Red");
		_html2colorname.put("#FF6347","Tomato");
		_html2colorname.put("#FF2400","Orange Red II");
		_html2colorname.put("#FF0000","Red");
		_html2colorname.put("#8B0000","Dark Red");
		_html2colorname.put("#8C1717","Scarlet");
		_html2colorname.put("#8E2323","Fire Brick II");
		_html2colorname.put("#B22222","Fire Brick");
		_html2colorname.put("#DC143C","Crimson");
		_html2colorname.put("#CD5C5C","Indian Red");
		_html2colorname.put("#F08080","Light Coral");
		_html2colorname.put("#FA8072","Salmon");
		_html2colorname.put("#E9967A","Dark Salmon");
		_html2colorname.put("#FF7F50","Coral");
		_html2colorname.put("#FFA07A","Light Salmon");
		_html2colorname.put("#FFF5EE","Seashell");
		_html2colorname.put("#FFE4E1","Misty Rose");
		_html2colorname.put("#FFC0CB","Pink");
		_html2colorname.put("#FFB6C1","Light Pink");
		_html2colorname.put("#FF6EC7","Neon Pink");
		_html2colorname.put("#FF69B4","Hot Pink");
		_html2colorname.put("#FF1CAE","Spicy Pink");
		_html2colorname.put("#FF1493","Deep Pink");
		_html2colorname.put("#000000","Black");
		_html2colorname.put("#2F4F4F","Dark Slate Gray");
		_html2colorname.put("#696969","Dim Gray");
		_html2colorname.put("#808080","Dark Gray");
		_html2colorname.put("#708090","Slate Gray");
		_html2colorname.put("#778899","Light Slate Gray");
		_html2colorname.put("#A9A9A9","Gray");
		_html2colorname.put("#C0C0C0","Silver");
		_html2colorname.put("#CDCDCD","Light Gray");
		_html2colorname.put("#D3D3D3","Very Light Gray");
		_html2colorname.put("#DCDCDC","Gainsboro");
		_html2colorname.put("#F8F8FF","Ghost White");
		_html2colorname.put("#FFFFFF","White");
		
	}
	public boolean isColorName(String x)
	{
		return _colorname2html.get(x.trim().replaceAll(" ","").toUpperCase()) != null;
	}
	public GlobalOptions()
	{
		try
		{
			BuildColor();
			_mapping = new TreeMap();
			parse(new FileInputStream("settings.ini"));
		}
		catch (Exception e)
		{
		}
	}
	
	public boolean isBoolean(String value)
	{
		String tval = value.toUpperCase();
		return (value.equals("TRUE") || value.equals("FALSE")) || (value.equals("T") || value.equals("F"));
	}
	public boolean isInteger(String value)
	{
		if(value.indexOf(".")>=0) return false;
		try
		{
			Integer.parseInt(value); 
			return true;
		}
		catch (Exception e)
		{
			return false;
		}
	}
	
	public boolean getBoolean(String key, boolean def)
	{
		Object o = get(key);
		if(o==null || !(o instanceof Boolean)) { set(key, new Boolean(def)); return def; }
		return ((Boolean)o).booleanValue();
	}
	public int getInteger(String key, int def)
	{
		Object o = get(key);
		if(o==null || !(o instanceof Integer)) { set(key, new Integer(def)); return def; }
		return ((Integer)o).intValue();
	}
	public double getDouble(String key, double def)
	{
		Object o = get(key);
		if(o==null || !(o instanceof Double)) { set(key, new Double(def)); return def; }
		return ((Double)o).doubleValue();
	}

	public String getString(String key, String def)
	{
		Object o = get(key);
		if(o==null || !(o instanceof String)) { set(key, def); return def; }
		return ((String)o);
	}
	public IntTriplet getIntTriplet(String key, int def_x, int def_y, int def_z)
	{
		IntTriplet def = new IntTriplet();
		def.x = def_x;def.y = def_y;def.z = def_z;
		Object o = get(key);
		if(o==null || !(o instanceof IntTriplet)) { set(key, def); return def; }
		return ((IntTriplet)o);
	}

	private void writeHeader(FileOutputStream FOS) throws IOException
	{
		byte[] b= new byte[2];
		b[0] = (byte)(255);
		b[1] = (byte)(254);
		FOS.write(b);
	}
	private void writeChar(FileOutputStream FOS, char x)  throws IOException
	{
		byte[] b= new byte[2];
		b[0] = (byte)(x % 256);
		b[1] = (byte)(x / 256);
		FOS.write(b);
	}
	private void writeStringLine(FileOutputStream FOS, String x)  throws IOException
	{
		for(int i = 0; i < x.length(); i++)
		{
			writeChar(FOS,x.charAt(i));
		}
		writeChar(FOS,(char)13);
		writeChar(FOS,(char)10);
	}
	public void save(String filename)
	{
		try
		{
			FileOutputStream FOS = new FileOutputStream(filename);
			writeHeader(FOS);
			writeStringLine(FOS, "!BNJ3 settings");
			
			 
			for(Iterator it = _mapping.keySet().iterator(); it.hasNext(); )
			{
				String key = (String) it.next();
				String val = _mapping.get(key).toString();
				writeStringLine(FOS, key + "=" + val);
			}
			FOS.close();
		}
		catch (Exception e)
		{
			System.out.println("huh");
		}
	}
	public boolean isDouble(String value)
	{
		try
		{
			Double.parseDouble(value);
			return true;
		}
		catch (Exception e)
		{
			return false;
		}
	}
	
	public boolean isIntTriplet(String v)
	{
		if(v.indexOf("<") >= 0 && v.indexOf(">") >= 0)
		{
			int k1 = v.indexOf(",");
			int k2 = v.indexOf(",", k1+1);
			int k3 = v.indexOf(",", k2+1);
			return (k2 > k1 && k3 < 0 && k1 >= 0);
		}
		return false;
	}
	
	public boolean isHTMLColor(String v)
	{
		if(isColorName(v)) return true;
		if(v.indexOf("#")<0) return false;
		if(v.length()<=6) return false;
		if(v.charAt(0)=='#') return true;
		return false;
	}
	private int hexv(char x)
	{
		switch(x)
		{
			case '0': return 0;
			case '1': return 1;
			case '2': return 2;
			case '3': return 3;
			case '4': return 4;
			case '5': return 5;
			case '6': return 6;
			case '7': return 7;
			case '8': return 8;
			case '9': return 9;
			case 'A': return 10;
			case 'B': return 11;
			case 'C': return 12;
			case 'D': return 13;
			case 'E': return 14;
			case 'F': return 15;
		}
		return 0;
	}
	public IntTriplet parseHTMLColor(String v)
	{
		IntTriplet IT = new IntTriplet();
		IT.x = hexv(v.charAt(1)) * 16 + hexv(v.charAt(2));
		IT.y = hexv(v.charAt(3)) * 16 + hexv(v.charAt(4));
		IT.z = hexv(v.charAt(5)) * 16 + hexv(v.charAt(6));
		return IT;
	}
	public IntTriplet parseTriplet(String v)
	{
		IntTriplet T = new IntTriplet();
		String r = v;
		int m = r.indexOf("<");
		r = r.substring(m+1);
		r = r.substring(0,r.indexOf(">"));
		String z = r.substring(0,r.indexOf(","));
		r = r.substring(z.length()+1);
		T.x = Integer.parseInt(z);
		z = r.substring(0,r.indexOf(","));
		r = r.substring(z.length()+1);
		T.y = Integer.parseInt(z);
		T.z = Integer.parseInt(r);
		return T;
	}
	public void mapvalue(String key, String value)
	{
		Object o;
		if(isHTMLColor(value))
		{
			if(isColorName(value))
			{
				IntTriplet T = parseHTMLColor((String)_colorname2html.get(value.trim().replaceAll(" ","").toUpperCase()));
				o = T;
			}
			else
			{
				IntTriplet T = parseHTMLColor(value.toUpperCase().trim());
				o = T;
				
			}
		}
		else if(isIntTriplet(value))
		{
			IntTriplet T = parseTriplet(value);
			o = T;
		}
		else if(isBoolean(value))
		{
			o = new Boolean(value.toLowerCase());
		}
		else if(isInteger(value))
		{
			o = new Integer(value);
		}
		else if(isDouble(value))
		{
			o = new Double(value);
		}
		else 
		{
			o = value;
		}
		//System.out.println("mapping " + key + " to " + o.toString() );
		set(key,o);
	}
	private void parseline(String x)
	{
		if (x.length() <= 1) return;
		if (x.charAt(0) == '!')
		{
			//System.out.println("comment: " + x);
			return;
		}
		try
		{
			int k = x.indexOf("=");
			String key = x.substring(0, k).trim();
			//				System.out.println("got key:" + key);
			String val = x.substring(k + 1).trim();
			//				System.out.println("got val:" + val);
			mapvalue(key, val);
		}
		catch (Exception e)
		{
			// report error on line X
		}
	}
	public void parse(String filename)
	{
		try
		{
			parse(new FileInputStream(filename));
		}
		catch (Exception e)
		{
			// eh, don't care?
		}
	}
	public void parse(InputStream IS)
	{
		try
		{
			int h = IS.read();
			int l = IS.read();
			boolean isunicode = false;
			boolean swap = false;
			if (l == 254 && h == 255)
			{
				isunicode = true;
			}
			if (l == 255 && h == 254)
			{
				isunicode = true;
				swap = true;
			}
			if (!isunicode)
			{
				return;
			}
			String buffer = "";
			while (IS.available() > 0)
			{
				if (swap)
				{
					h = IS.read();
					l = IS.read();
				}
				else
				{
					l = IS.read();
					h = IS.read();
				}
				char code = (char) (short) (h * 256 + l);
				//System.out.println(" " + h + " : " + l);
				if (code == 10)
				{
					parseline(buffer.trim());
					buffer = "";
				}
				if (code >= 30)
				{
					buffer += code;
				}
			}
			parseline(buffer.trim());
			IS.close();
		}
		catch (Exception e)
		{
			e.printStackTrace();
			System.out.println("failure?");
		}
	}
}