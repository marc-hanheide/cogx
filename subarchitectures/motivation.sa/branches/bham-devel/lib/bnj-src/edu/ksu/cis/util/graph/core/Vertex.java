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
 */package edu.ksu.cis.util.graph.core;
/*!
 * \file Vertex.java
 * \author Jeffrey M. Barber
 */
public class Vertex
{
	private String	_Name;
	private int[]	_VisualAttrib;
	private int		_Location;
	private Object	_Object;
	/*! Get the data object
	 *  \param[in] o the data object
	 */
	public void setObject(Object o)
	{
		_Object = o;
	}
	/*! Get the data object
	 * \return the data object
	 */
	public Object getObject()
	{
		return _Object;
	}
	/*! Construct a vertex with a name
	 * \param[in] name - the name of vertex
	 */
	public Vertex(String name)
	{
		_Name = name;
		_VisualAttrib = new int[6];
		_VisualAttrib[0] = 0;
		_VisualAttrib[1] = 0;
		_VisualAttrib[2] = 0;
		_VisualAttrib[3] = 0;
		_VisualAttrib[4] = 0;
		_VisualAttrib[5] = 0;
		_Location = -1;
		_Object = null;
	}
	/*! Get the X Coordinate
	 * \return the x position
	 */
	public int getx()
	{
		return _VisualAttrib[0];
	}
	/*! Get the Y Coordinate
	 * \return the y position
	 */
	public int gety()
	{
		return _VisualAttrib[1];
	}
	/*! Translate the vertex
	 * \param[in] x the x offset
	 * \param[in] y the y offset
	 */
	public void translate(int x, int y)
	{
		_VisualAttrib[0] += x;
		_VisualAttrib[1] += y;
	}
	/*! Get the name
	 * \return the name of the vertex
	 */
	public String getName()
	{
		return _Name;
	}
	/*! Change the Vertex's name
	 * \param name new name of the vertex
	 */
	public void setName(String name)
	{
		_Name = name;
	}
	/*! Change a Vertex's attribute register 
	 * \param[in] idx The index of the vertex register
	 * \param[in] val The register's new value
	 */
	public void setAttrib(int idx, int val)
	{
		_VisualAttrib[idx] = val;
	}
	/*! Query a Vertex' attribute register 
	 * \param[in] idx The index of the vertex register
	 * \return the value of the register
	 */
	public int getAttrib(int idx)
	{
		return _VisualAttrib[idx];
	}
	/*! copy the vertex
	 * \return  a copy of the vertex
	 */
	public Vertex copy()
	{
		Vertex V = new Vertex(_Name);
		V.translate(_VisualAttrib[0], _VisualAttrib[1]);
		V.setAttrib(2, _VisualAttrib[2]);
		V.setAttrib(3, _VisualAttrib[3]);
		V.setAttrib(4, _VisualAttrib[4]);
		V.setAttrib(5, _VisualAttrib[5]);
		V.setLOC(-1);
		V.setObject(_Object);
		return V;
	}
	/*! the location in the store of the vertex, also denotes if bound to a graph
	 * \return the location in the store
	 */
	public int loc()
	{
		return _Location;
	}
	/*! sets the store location
	 * \note
	 * \par
	 * 	DO NOT USE
	 * \param[in] x the new location store
	 */
	public void setLOC(int x)
	{
		_Location = x;
	}
}