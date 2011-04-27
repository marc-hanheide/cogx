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
 * file: Moralization.java
 * 
 * @author Jeffrey M. Barber
 */
public class Moralization extends Algorithm implements Visitor
{
	private boolean		_vis;
	private OrderedList	_AddedEgdes;
	private OrderedList	_ParentNodes;
	public Moralization()
	{_Name = "Moralization";	}		
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.util.graph.visualization.Algorithm#execute()
	 */
	public void execute()
	{
		_vis = super.canVisualize();
		_AddedEgdes = new OrderedList();
		_ParentNodes = new OrderedList();
		if(_vis)
		{
			VC().beginTransaction();
			VC().pushAndApplyOperator(new Annotation("Moralization"));
			VC().pushAndApplyOperator(new CodePageSelect(0));
			VC().pushAndApplyOperator(new CodePageSelectLine(0));
			VC().pushAndApplyOperator(new NewColorLegend());
			VC().pushAndApplyOperator(new ColorLegendMap(0,"No activity"));
			VC().pushAndApplyOperator(new ColorLegendMap(1,"active"));
			VC().pushAndApplyOperator(new ColorLegendMap(2,"first parent of active"));
			VC().pushAndApplyOperator(new ColorLegendMap(3,"second parent of active"));
			
		}
		_Graph.apply(this, true, false);
		if (_vis)
		{
			VC().pushAndApplyOperator(new TempEdgeFlush());
			VC().pushAndApplyOperator(new CodePageSelectLine(10));
			for (int i = 0; i < _AddedEgdes.size(); i++)
			{
				Edge E = (Edge) _AddedEgdes.get(i);
				_Graph.addEdge(E);
				Vertex V = (Vertex) _ParentNodes.get(i);

				GraphBatch B = new GraphBatch();
				B.add(new VertexColor(E.s(), 2));
				B.add(new VertexColor(E.d(), 2));
				super.VC().pushAndApplyOperator(B);
				super.VC().pushAndApplyOperator(new EdgeCreate(E));
				B = new GraphBatch();
				B.add(new VertexColor(E.s(), 0));
				B.add(new VertexColor(E.d(), 0));
				super.VC().pushAndApplyOperator(B);
				_Graph = super.VC().getFrame();
			}
			VC().pushAndApplyOperator(new CodePageSelectLine(-1));
		}
		else
		{
			for (int i = 0; i < _AddedEgdes.size(); i++)
			{
				Edge E = (Edge) _AddedEgdes.get(i);
				_Graph.addEdge(E);
			}
		}
		if (_vis) VC().commitTransaction();
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.util.graph.viistors.Visitor#onVertex(edu.ksu.cis.util.graph.core.Vertex)
	 */
	/*
	 * 		 c[0] = "set `e`";
		 c[1] = "for each v in G";
		 c[2] = "   for each (`p1`,`v`) in G";
		 c[3] = "      for each (`p2`,`v) in G";
		 c[4] = "         if (`w1`,`w2`) is not in G.edges and (`w2`,`w1`) is not in G.edges then";
		 c[5] = "         	`e` = `e` union (`w1`,`w2`)";
		 c[6] = "         end";
		 c[7] = "      next";
		 c[8] = "   next";
		 c[9] = "next";
		c[10] = "G.edges = G.edges union `e`";
	 */
	public boolean onVertex(Vertex V)
	{
		Vertex[] Parents = _Graph.getParents(V);
		int N = Parents.length;
//		if (_vis) super.VC().pushAndApplyOperator(new VertexColor(V, new Color(100, 100, 255)));
		// for each pair of parents
		if(_vis)
		{
			VC().pushAndApplyOperator(new VertexColor(V, 1));
			VC().pushAndApplyOperator(new CodePageSelectLine(1));
			VC().pushAndApplyOperator(new CodePageUpdateEnvironment("v",V.getName()));
			for (int i = 0; i < N; i++)
			{
				VC().pushAndApplyOperator(new VertexColor(Parents[i], 2));
				VC().pushAndApplyOperator(new CodePageSelectLine(2));
				VC().pushAndApplyOperator(new CodePageUpdateEnvironment("p1",Parents[i].getName()));
				for (int j = i+1; j < N; j++)
				{
					VC().pushAndApplyOperator(new VertexColor(Parents[j], 3));
					VC().pushAndApplyOperator(new CodePageSelectLine(3));
					VC().pushAndApplyOperator(new CodePageUpdateEnvironment("p2",Parents[j].getName()));
					if (i != j)
					{
						// they are not connected?
						VC().pushAndApplyOperator(new CodePageSelectLine(4));
						if (_Graph.getConnectedness(Parents[i], Parents[j]) == 0)
						{
							VC().pushAndApplyOperator(new CodePageSelectLine(5));
							if(_vis) _ParentNodes.add(V);
							// connect them (fixed, in the list... 7/6 - JMB)
							Edge E = new Edge(Parents[i], Parents[j]);
							E.setDirected(false);
							//E.setDirected(false);
							VC().pushAndApplyOperator(new TempEdgeCreate(E));

							_AddedEgdes.add(E);
						}
						VC().pushAndApplyOperator(new CodePageSelectLine(6));
					}
				VC().pushAndApplyOperator(new VertexColor(Parents[j], 0));
				VC().pushAndApplyOperator(new CodePageSelectLine(7));
				}
			VC().pushAndApplyOperator(new VertexColor(Parents[i], 0));
			VC().pushAndApplyOperator(new CodePageSelectLine(8));
			}
		VC().pushAndApplyOperator(new VertexColor(V, 0));
		VC().pushAndApplyOperator(new CodePageSelectLine(9));
		}
		else
		{
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < N; j++)
				{
					if (i != j)
					{
						// they are not connected?
						if (_Graph.getConnectedness(Parents[i], Parents[j]) == 0)
						{
							if(_vis) _ParentNodes.add(V);
							// connect them (fixed, in the list... 7/6 - JMB)
							Edge E = new Edge(Parents[i], Parents[j]);
							//E.setDirected(false);
							_AddedEgdes.add(E);
						}
					}
				}
			}
		}
//		if (_vis) super.VC().pushAndApplyOperator(new VertexColor(V, new Color(100, 100, 100)));
		return true;
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.util.graph.viistors.Visitor#onEdge(edu.ksu.cis.util.graph.core.Edge)
	 */
	public boolean onEdge(Edge E)
	{
		return false;
	}
}