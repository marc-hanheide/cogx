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
import edu.ksu.cis.util.data.OrderedList;
import edu.ksu.cis.util.graph.visualization.*;
import edu.ksu.cis.util.graph.visualization.operators.*;
import edu.ksu.cis.util.graph.core.*;
/**
 * file: FillIn.java
 * 
 * @author Jeffrey M. Barber
 */
public class FillIn extends Algorithm
{
	private Vertex[]	alphainv;
	private int[]		alpha;
	public FillIn()
	{_Name = "Fill-in Computation";	}			
	/**
	 * [CONSTRUCTUR]
	 * 
	 * @param a
	 * @param ainv
	 */
	public FillIn(int[] a, Vertex[] ainv)
	{
		alpha = a;
		alphainv = ainv;
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.util.graph.visualization.Algorithm#execute()
	 */
	public void execute()
	{
		boolean _vis = canVisualize();
		if (_vis)
			{
			VC().beginTransaction();
		if (_vis) VC().pushAndApplyOperator(new Annotation("Fill-In"));
			VC().pushAndApplyOperator(new NewColorLegend());
			VC().pushAndApplyOperator(new ColorLegendMap(0,"No activity"));
			VC().pushAndApplyOperator(new ColorLegendMap(7,"w"));
			VC().pushAndApplyOperator(new ColorLegendMap(8,"x"));
			VC().pushAndApplyOperator(new ColorLegendMap(9,"new edge"));
			
			
			VC().pushAndApplyOperator(new CodePageSelect(3));
			VC().pushAndApplyOperator(new CodePageSelectLine(0));
			VC().pushAndApplyOperator(new CodePageSelectLine(1));
			VC().pushAndApplyOperator(new CodePageSelectLine(2));
			VC().pushAndApplyOperator(new CodePageSelectLine(3));
			VC().pushAndApplyOperator(new CodePageSelectLine(4));
			VC().pushAndApplyOperator(new CodePageSelectLine(5));
			VC().pushAndApplyOperator(new CodePageSelectLine(6));
			VC().pushAndApplyOperator(new CodePageSelectLine(7));
			VC().pushAndApplyOperator(new CodePageSelectLine(8));
			VC().pushAndApplyOperator(new CodePageSelectLine(9));
			VC().pushAndApplyOperator(new CodePageSelectLine(10));
			}

		int n = _Graph.getNumberOfVertices();
		Vertex f[] = new Vertex[n];
		int index[] = new int[n];
		OrderedList EunionFalpha = new OrderedList();
		int i;
		Vertex v, w, x;
		for (i = n; i >= 1; i--)
		{
			if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("i","" + i));
			
			w = alphainv[i];
			if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("w","" + w.getName()));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(11));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(12));
			if(_vis) super.VC().pushAndApplyOperator(new VertexColor(w,7));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(13));
			f[w.loc()] = w;
			index[w.loc()] = i;
			Vertex[] par = _Graph.getParents(w);
			for (int k = 0; k < par.length; k++)
			{
				v = par[k];
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(14));
				if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("v","" + v.getName()));
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(15));
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(16));
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(17));
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(18));
				x = v;
				if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("x","" + x.getName()));
				if(_vis) super.VC().pushAndApplyOperator(new VertexColor(x,8));
				while (index[x.loc()] > i)
				{
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(19));
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(20));
					index[x.loc()] = i;
					Edge E = new Edge(x, w);
					E.setDirected(false);
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(21));
					if (_vis) VC().pushAndApplyOperator(new TempEdgeCreate(E));
					EunionFalpha.add(E);
					if(_vis) super.VC().pushAndApplyOperator(new VertexColor(x,0));
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(22));
					x = f[x.loc()];
					if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("x","" + x.getName()));
					if(_vis) super.VC().pushAndApplyOperator(new VertexColor(x,8));
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(23));
				}
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(24));
				if (f[x.loc()] == x)
					{
						if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(25));
						f[x.loc()] = w;
					}
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(26));
				if(_vis) super.VC().pushAndApplyOperator(new VertexColor(x,0));
			}
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(27));
			if(_vis) super.VC().pushAndApplyOperator(new VertexColor(w,0));
		}
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(28));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(29));
		if (_vis) VC().pushAndApplyOperator(new TempEdgeFlush());
		for (int k = 0; k < EunionFalpha.size(); k++)
		{
			Edge E = (Edge) EunionFalpha.get(k);
			if (_Graph.getConnectedness(E.s(), E.d()) == 0)
			{
				if (_vis)
				{
					GraphBatch B = new GraphBatch();
					B.add(new VertexColor(E.s(), 9));
					B.add(new VertexColor(E.d(), 9));
					super.VC().pushAndApplyOperator(new EdgeCreate(E));
					B = new GraphBatch();
					B.add(new VertexColor(E.s(), 0));
					B.add(new VertexColor(E.d(), 0));
				}
				else
				{
					_Graph.addEdge(E);
				}
			}
		}
		if (_vis) VC().commitTransaction();
	}
}
/*
 * for (int k = _Graph.begin(); _Graph.end(k); k++) { v = _Graph.get(k); if
 * (_Graph.getConnectedness(v, w) != 0 && alpha[v.loc()] > i) { x = v; while
 * (index[x.loc()] > i) { index[x.loc()] = i; Edge E = new Edge(x, w);
 * E.setDirected(false); System.out.println("fillinmine: connecting:" +
 * E.s().getName() + " to " + E.d().getName()); EunionFalpha.add(E); x =
 * f[x.loc()]; } if (f[x.loc()] == x) f[x.loc()] = w; } }
 */