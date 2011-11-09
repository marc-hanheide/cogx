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
 */package edu.ksu.cis.util.graph.algorithms;
import edu.ksu.cis.util.graph.core.Vertex;
import edu.ksu.cis.util.graph.visualization.Algorithm;
import edu.ksu.cis.util.graph.visualization.Annotation;
import edu.ksu.cis.util.graph.visualization.operators.VertexChangeOrderIndex;
/**
 * file: TopologicalSort.java
 * 
 * @author Jeffrey M. Barber
 */
public class TopologicalSort extends Algorithm
{
	private boolean[]	_mark;
	public int[]		alpha;
	private int			_pos;
	private boolean		_vis;

	public TopologicalSort()
	{_Name = "Topological Sort";	}

	private void InitMark()
	{
		for (int i = 0; i < _mark.length; i++)
		{
			_mark[i] = false;
		}
	}
	private void search(Vertex V)
	{
		if (_mark[V.loc()]) return;
		_mark[V.loc()] = true;
		Vertex[] Children = _Graph.getChildren(V);
		for (int i = 0; i < Children.length; i++)
		{
			search(Children[i]);
		}
		if(_vis) VC().pushAndApplyOperator(new VertexChangeOrderIndex(V,alpha.length - _pos));
		alpha[_pos] = V.loc();
		_pos--;
	}
	public void execute()
	{
		_vis = false;
		if (_Graph == null) return;
		if(VC()!=null) _vis = true;
		if(_vis) VC().beginTransaction();
		if (_vis) VC().pushAndApplyOperator(new Annotation("Topological Sort"));
		Vertex[] V = _Graph.getVertices();
		_mark = new boolean[V.length];
		InitMark();
		alpha = new int[V.length];
		_pos = V.length - 1;
		for (int i = 0; i < V.length; i++)
		{
			if (!_mark[i])
			{
				search(V[i]);
			}
		}
		if(_vis) VC().commitTransaction();
	}
}