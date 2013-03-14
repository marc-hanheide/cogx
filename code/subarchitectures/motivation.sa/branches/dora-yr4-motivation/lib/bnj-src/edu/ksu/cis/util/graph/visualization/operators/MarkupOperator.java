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
 */package edu.ksu.cis.util.graph.visualization.operators;
import java.util.ArrayList;
import edu.ksu.cis.util.graph.core.Edge;
import edu.ksu.cis.util.graph.visualization.Markup;
/*!
 * \file MarkupOperator.java
 * \author Jeffrey M. Barber
 */
public class MarkupOperator
{
	private Markup	_M;
	private Edge	_edgeTrack;
	/*! a marking at the midpoint of the edge E
	 * @param[in] E the edge to track
	 * @param[in] msg the message
	 */
	public MarkupOperator(Edge E, String msg)
	{
		_M = new Markup();
		_edgeTrack = E;
		_M.msg = msg;
	}
	/*! a marking at (x,y) of msg
	 * \param[in] x the x comp
	 * \param[in] y the y comp
	 * \param[in] msg the message
	 */
	public MarkupOperator(int x, int y, String msg)
	{
		_edgeTrack = null;
		_M = new Markup();
		_M.x = x;
		_M.y = y;
		_M.msg = msg;
	}
	/*! apply this markup (add)
	 * @param al
	 */
	public void apply(ArrayList al)
	{
		if (_edgeTrack != null)
		{
			_M.x = (_edgeTrack.s().getx() + _edgeTrack.d().getx()) / 2;
			_M.y = (_edgeTrack.s().gety() + _edgeTrack.d().gety()) / 2;
		}
		al.add(_M);
	}
	/*! apply the inverse of this markup (remove)
	 * @param al
	 */
	public void applyInverse(ArrayList al)
	{
		al.remove(_M);
	}
}