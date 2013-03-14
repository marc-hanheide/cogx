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
import edu.ksu.cis.util.graph.core.Graph;
import edu.ksu.cis.util.graph.core.Vertex;
import edu.ksu.cis.util.graph.visualization.Algorithm;
/**
 * file: DetectCycles.java
 * 
 * @author Jeffrey M. Barber
 */
public class DetectCycles extends Algorithm
{
	public boolean		hasCycles;
	private boolean[]	Mark;
	private Vertex		Goal;
	public DetectCycles()
	{_Name = "Detect Graph Cycles";	}		
	private void InitMark()
	{
		for (int i = 0; i < Mark.length; i++)
		{
			Mark[i] = false;
		}
	}
	private void search(Graph G, Vertex V)
	{
		if (Mark[V.loc()]) return;
		Mark[V.loc()] = true;
		Vertex[] Children = G.getChildren(V);
		for (int i = 0; i < Children.length; i++)
		{
			if (Children[i].loc() == Goal.loc())
			{
				hasCycles = true;
				return;
			}
			else
			{
				search(G, Children[i]);
			}
		}
	}
	public void execute()
	{
		hasCycles = false;
		if (_Graph == null) return;
		Graph hack = _Graph.copy();
		RemoveDirectionality RD = new RemoveDirectionality();
		RD.setGraph(hack);
		RD.execute();
		hack = RD.getGraph();
		Vertex[] V = hack.getVertices();
		Mark = new boolean[V.length];
		for (int i = 0; i < V.length; i++)
		{
			InitMark();
			Goal = V[i];
			search(hack, V[i]);
			;
			if (hasCycles) return;
		}
	}
}