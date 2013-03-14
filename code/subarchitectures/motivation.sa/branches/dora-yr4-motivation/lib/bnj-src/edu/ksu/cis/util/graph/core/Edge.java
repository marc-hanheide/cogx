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
 * \file Edge.java
 * \author Jeffrey M. Barber
 */
public class Edge
{
	private Vertex	_Source;
	private Vertex	_Destination;
	private boolean	_IsDirected;
	/*! Construct an edge for communication
	 * \param s - the source vertex
	 * \param d - the destination vertex
	 */
	public Edge(Vertex s, Vertex d)
	{
		_Source = s;
		_Destination = d;
		_IsDirected = true;
	}
	/*! Slight of hand accessor for src
	 * \return the source vertex
	 */
	public Vertex s()
	{
		return _Source;
	}
	/*! Slight of hand accessor for dest
	 * \return the destination vertex
	 */
	public Vertex d()
	{
		return _Destination;
	}
	/*! Set how the edge is connected
	 *\param dir
	 */
	public void setDirected(boolean dir)
	{
		_IsDirected = dir;
	}
	/*!
	 * \return how the edge is connected, true => (src,dst) | false => (src,dst) and (dst,src)
	 */
	public boolean isDirected()
	{
		return _IsDirected;
	}
	/*! Invert this Edge
	 */
	public void invert()
	{
		Vertex temp = _Source;
		_Source = _Destination;
		_Destination = temp;
	}
	/*! Get the oppposite vertex of A
	 * \param[in] A the vertex to hold onto
	 * \return the opposite vertex of A
	 */
	public Vertex opposite(Vertex A)
	{
		if (A == _Source) return _Destination;
		if (A == _Destination) return _Source;
		return null;
	}
	/*! Get the inversion of this edge
	 * \return the inverted edge
	 */
	public Edge getInvertedEdge()
	{
		return new Edge(_Destination, _Source);
	}
	/*! copy() this edge
	 * \return a copy of the edge
	 */
	public Edge copy()
	{
		Edge E = new Edge(_Source, _Destination);
		E.setDirected(_IsDirected);
		return E;
	}
}