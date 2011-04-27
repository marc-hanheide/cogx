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
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.HashMap;
public class LanguageUnicodeParser
{
	static LanguageUnicodeParser	instance	= new LanguageUnicodeParser();
	public boolean					debugmode	= true;
	static public LanguageUnicodeParser getInstance()
	{
		return instance;
	}
	public LanguageUnicodeParser()
	{
		try
		{
			_mapping = new HashMap();
			parse(new FileInputStream("default.ini"));
		}
		catch (Exception e)
		{
			// eh, don't care?
		}
	}
	private HashMap	_mapping;
	private void mapvalue(String key, String val)
	{
		_mapping.put(key, val);
	}
	public String get(String key)
	{
		String v = (String) _mapping.get(key);
		if (v == null)
		{
			if (debugmode) System.out.println(key + " = ?");
			return ".ini bad" + key;
		}
		return v;
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
			//			System.out.println("got key:" + key);
			String val = x.substring(k + 1).trim();
			//			System.out.println("got val:" + val);
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