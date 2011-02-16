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
import java.util.*;
import edu.ksu.cis.util.graph.core.Edge;
import edu.ksu.cis.util.graph.core.Graph;
import edu.ksu.cis.util.graph.core.Vertex;
import edu.ksu.cis.util.graph.visualization.Algorithm;
import edu.ksu.cis.util.graph.visualization.Annotation;
import edu.ksu.cis.util.graph.visualization.operators.CodePageSelect;
import edu.ksu.cis.util.graph.visualization.operators.CodePageSelectLine;
import edu.ksu.cis.util.graph.visualization.operators.CodePageUpdateEnvironment;
import edu.ksu.cis.util.graph.visualization.operators.ColorLegendMap;
import edu.ksu.cis.util.graph.visualization.operators.Delay;
import edu.ksu.cis.util.graph.visualization.operators.EdgeCreate;
import edu.ksu.cis.util.graph.visualization.operators.GraphBatch;
import edu.ksu.cis.util.graph.visualization.operators.MarkupOperator;
import edu.ksu.cis.util.graph.visualization.operators.NewColorLegend;
import edu.ksu.cis.util.graph.visualization.operators.SwitchGraph;
import edu.ksu.cis.util.graph.visualization.operators.VertexColor;
import edu.ksu.cis.util.graph.visualization.operators.VertexCreate;
/**
 * file: BuildCliqueTree.java
 * 
 * @author Jeffrey M. Barber
 */
public class BuildCliqueTree extends Algorithm
{
	private Graph		_CliqueTree;
	private Vertex[]	_CliquesSet;
	private HashSet[]	_Cliques_S;
	//private HashSet[] _Cliques_R;
	private HashSet[]	_CliquesBaseNodes;
	public BuildCliqueTree()
	{_Name = "Build Clique Tree";	}		
	public Graph getCliqueTree()
	{
		return _CliqueTree;
	}
	public int getNumberOfCliques()
	{
		return _CliquesSet.length;
	}
	public Vertex getClique(int i)
	{
		return _CliquesSet[i];
	}
	public HashSet getCliqueSet(int i)
	{
		return (HashSet) _CliquesSet[i].getObject();
	}
	public HashSet getCliqueS(int i)
	{
		return _Cliques_S[i];
	}
	/*
	 * public HashSet getCliqueR(int i) { return _Cliques_R[i]; }
	 */
	public HashSet getCliqueBases(int i)
	{
		return _CliquesBaseNodes[i];
	}
	public Vertex[] getChildren(int i)
	{
		return _CliqueTree.getChildren(_CliquesSet[i]);
	}
	public void execute()
	{
		boolean _vis = false;
		if(VC()!=null) VC().beginTransaction();
		if(VC()!=null) _vis = true;
		if (_Graph.getVertices().length == 0) return;
		Graph original = _Graph.copy();
		Graph mimic = _Graph.copy();
		
		if(_vis)
		{
			VC().pushAndApplyOperator(new SwitchGraph(_Graph, mimic));
			_Graph = VC().getFrame();
		}
		else
		{
			_Graph = mimic;
		}
		
		//System.out.println("" + _Graph);
		TriangulationByMaxCardinalitySearch TMCS = new TriangulationByMaxCardinalitySearch();
		TMCS.setGraph(_Graph);
		TMCS.setVisualization(VC());
		TMCS.execute();

		if (_vis) VC().pushAndApplyOperator(new Annotation("Build Clique Tree : Finding Cliques"));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelect(4));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(0));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(1));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(2));
		if(_vis)
		{
			VC().pushAndApplyOperator(new NewColorLegend());
			VC().pushAndApplyOperator(new ColorLegendMap(0,"No activity"));
			VC().pushAndApplyOperator(new ColorLegendMap(10,"scan"));
			VC().pushAndApplyOperator(new ColorLegendMap(11,"base"));
			VC().pushAndApplyOperator(new ColorLegendMap(12,"parent"));
			VC().pushAndApplyOperator(new ColorLegendMap(13,"accpt"));
		}
		_Graph = TMCS.getGraph();

		//if(VC()!=null) VC().commitTransaction();		
		
		// assume the graph is undirected moral and triangulated

		LinkedList cliques = new LinkedList(); // List of cliques, ordered
		Vertex[] alphainv = TMCS.getAlphaInverse();
		int[] alpha = TMCS.getAlpha();
		Vertex[] vertices = _Graph.getVertices();
		Vertex T;
		_CliqueTree = new Graph();
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(3));

		for (int i = vertices.length; i > 0; i--)
		{
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(4));
			if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("i","" + i));
			Vertex v = alphainv[i];
			if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("v",v.getName()));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(5));
			Vertex[] parents = _Graph.getParents(v);
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(6));
			HashSet clique = new HashSet();
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(7));
			clique.add(v);
			if (_vis) VC().pushAndApplyOperator(new VertexColor(v,11));
			
			//System.out.println("adding 2 click ("+i+" :)"+v.getName());
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(8));
			if (parents != null)
			{
				for (int j = 0; j < parents.length; j++)
				{
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(9));
					Vertex p = parents[j];
					if (_vis) VC().pushAndApplyOperator(new VertexColor(p,12));
					if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("w",p.getName()));
					int order = alpha[p.loc()];
					if (order <= i)
					{
						if (_vis) VC().pushAndApplyOperator(new VertexColor(p,13));
						if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(10));
						clique.add(p);
						//System.out.println("adding 2z click ("+i+"
						// :)"+p.getName());
					}
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(11));
					if (_vis) VC().pushAndApplyOperator(new VertexColor(p,0));
				}
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(12));
			}
			if (_vis) VC().pushAndApplyOperator(new VertexColor(v,0));
			cliques.add(clique);
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(13));
		}
		
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(14));


		
		//		Rescan for containment
		HashSet[] cliqueSet = (HashSet[]) cliques.toArray(new HashSet[0]);
		int max = cliqueSet.length;
		cliques.clear(); // empty the clique list
		for (int i = 0; i < max; i++)
		{
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(15));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(16));
			//System.out.println("@"+i);
			boolean isContained = false;
			if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("contain","false"));
			for (int j = 0; j < max; j++)
			{
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(17));
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(18));
				if (i != j && cliqueSet[j].containsAll(cliqueSet[i]))
				{
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(19));
					if (_vis) VC().pushAndApplyOperator(new CodePageUpdateEnvironment("contain","true"));
					isContained = true;
					break;
				}
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(20));
			}
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(21));
			if (!isContained)
			{
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(22));
				//cliques.add(cliqueSet[i]);
				String cliqueName = "{";
				boolean first = true;
				for (Iterator z = cliqueSet[i].iterator(); z.hasNext();)
				{
					Vertex node = (Vertex) z.next();
					if (!first) cliqueName += ",";
					first = false;
					cliqueName += node.getName();
				}
				cliqueName+="}";
				//System.out.println("clique: " + cliqueName);
				Vertex clique = new Vertex(cliqueName);
				clique.setObject(cliqueSet[i]);
				
				if(_vis)
				{
					int x = 0;
					int y = 0;
					int n = 0;
					for (Iterator z = cliqueSet[i].iterator(); z.hasNext();)
					{
						Vertex node = (Vertex) z.next();
						x += node.getx();
						y += node.gety();
						n++;
						VC().pushAndApplyOperator(new VertexColor(node,10));
					}
					clique.setAttrib(0,x/n);
					clique.setAttrib(1,y/n);
					
					VC().pushAndApplyOperator(new Delay("delay_vis_build_clique_tree_waiton_clique_highlight", 1000));
					VC().pushAndApplyOperator(new SwitchGraph(_Graph,_CliqueTree));
					VC().pushAndApplyOperator(new VertexCreate(clique));
					VC().pushAndApplyOperator(new Delay("delay_vis_build_clique_tree_switch", 1000));
					VC().pushAndApplyOperator(new SwitchGraph(_CliqueTree,_Graph));
					//_CliqueTree.addVertex(clique);
					GraphBatch B = new GraphBatch();
					for (Iterator z = cliqueSet[i].iterator(); z.hasNext();)
					{
						Vertex node = (Vertex) z.next();
						B.add(new VertexColor(node,0));
					}					
					VC().pushAndApplyOperator(B);
					
					//VC().pushAndApplyOperator(new SwitchGraph(_CliqueTree,mimic));
				}
				else
				{
					_CliqueTree.addVertex(clique);	
				}
				//Clique clique = new Clique(this, cliqueSet[i]);
				//addNode(clique);
				cliques.addFirst(clique);
			}
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(23));
		}
		if(_vis)
			{
			VC().pushAndApplyOperator(new SwitchGraph(_Graph,_CliqueTree));
			VC().pushAndApplyOperator(new Delay("delay_vis_build_clique_tree_switch", 1000));
			}
		

		if (_vis) VC().pushAndApplyOperator(new Annotation("Build Clique Tree : Connecting Cliques"));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelect(5));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(0));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(1));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(2));
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(3));
		if(_vis)
		{
			VC().pushAndApplyOperator(new NewColorLegend());
			VC().pushAndApplyOperator(new ColorLegendMap(0,"No activity"));
			VC().pushAndApplyOperator(new ColorLegendMap(20,"scan"));
			VC().pushAndApplyOperator(new ColorLegendMap(21,"parent"));
			VC().pushAndApplyOperator(new ColorLegendMap(22,"accept"));
		}
		_CliquesSet = (Vertex[]) cliques.toArray(new Vertex[0]);
		max = _CliquesSet.length;
		_Cliques_S = new HashSet[max];
		//_Cliques_R = new HashSet[max];
		HashSet union = new HashSet();
		union.addAll((HashSet) _CliquesSet[0].getObject()); // union = Clq_0 U
		// ... U Clq_i-1
		if (_vis) VC().pushAndApplyOperator(new VertexColor(_CliquesSet[0],11));
		Hashtable parentTable = new Hashtable();
		_CliquesBaseNodes = new HashSet[_CliquesSet.length];
		_CliquesBaseNodes[0] = new HashSet();
		_Cliques_S[0] = new HashSet();
		//_Cliques_R[0] = new HashSet();
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(4));
		for (int i = 1; i < max; i++)
		{
			if (_vis) VC().pushAndApplyOperator(new VertexColor(_CliquesSet[i],20));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(5));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(6));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(7));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(8));
			_CliquesBaseNodes[i] = new HashSet();
			HashSet s = new HashSet();
			HashSet nodeSet = (HashSet) (_CliquesSet[i].getObject());
			s.addAll(nodeSet);
			s.retainAll(union); // S = Clq_i \cup union
//			HashSet r = new HashSet();
//			r.addAll(nodeSet);
//			r.removeAll(s);
			_Cliques_S[i] = s;
			//_Cliques_R[i] = r;
			//cliqueSet[i].setS(s);
			union.addAll(nodeSet);
			//System.out.println("S for " + _CliquesSet[i].getName());
			/*
			for (Iterator itS = s.iterator(); itS.hasNext();)
			{
				Vertex o = (Vertex) itS.next();
				//System.out.println(" e: " + o.getName());
			}
			*/
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(9));
			for (int j = i - 1; j >= 0; j--)
			{
				HashSet csj = (HashSet) (_CliquesSet[j].getObject());
				//if (cliqueSet[j]..containsAll(s))
				if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(10));
				if (_vis) VC().pushAndApplyOperator(new VertexColor(_CliquesSet[j],21));
				if (csj.containsAll(s))
				{
					if (_vis) VC().pushAndApplyOperator(new VertexColor(_CliquesSet[j],22));
					if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(11));
					parentTable.put(_CliquesSet[i], _CliquesSet[j]);
				}
			}
			if (_vis) VC().pushAndApplyOperator(new VertexColor(_CliquesSet[i],0));
			if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(12));
		}
		if (_vis) VC().pushAndApplyOperator(new CodePageSelectLine(13));
		
		
		if(!_vis)
		{
			for (Enumeration e = parentTable.keys(); e.hasMoreElements();)
			{
				Vertex child = (Vertex) e.nextElement();
				Vertex parent = (Vertex) parentTable.get(child);
				_CliqueTree.addDirectedEdge(parent, child);
				//			_CliqueTree.addDirectedEdge(child, parent);
				//addEdge(parent, child);
			}
		}
		else
		{
			for (Enumeration e = parentTable.keys(); e.hasMoreElements();)
			{
				Vertex child = (Vertex) e.nextElement();
				Vertex parent = (Vertex) parentTable.get(child);
				HashSet SP = (HashSet)parent.getObject();  
				HashSet SC = (HashSet)child.getObject();
				HashSet setsmsg = _Cliques_S[child.loc()];
					//new HashSet();
				//setsmsg.addAll(SP);
				//setsmsg.retainAll(SC);
				String msg = "{";
				boolean first = true;
				for(Iterator it = setsmsg.iterator(); it.hasNext(); )
				{
					Vertex V = (Vertex) (it.next());
					if(!first) msg+=",";
					first = false;
					msg+=V.getName();
				}
				msg+="}";
				//_CliqueTree.addDirectedEdge(parent, child);
				Edge E = new Edge(parent,child);
				VC().pushAndApplyOperator(new EdgeCreate(E));
				VC().pushAndApplyOperator(new MarkupOperator(E,msg));
				//			_CliqueTree.addDirectedEdge(child, parent);
				//addEdge(parent, child);
			}			
		}
		
		Vertex[] orig_vert = original.getVertices();
		for (int z = 0; z < orig_vert.length; z++)
		{
			Vertex v = orig_vert[z];
			Vertex[] p = original.getParents(v);
			HashSet total = new HashSet();
			total.add(vertices[v.loc()]);
			if (p != null) for (int zz = 0; zz < p.length; zz++)
				total.add(vertices[p[zz].loc()]);
			for (int csi = 0; csi < max; csi++)
			{
				HashSet set = (HashSet) _CliquesSet[csi].getObject();
				if (set.containsAll(total))
				{
					//System.out.println("adding base node " + v.getName() + "
					// to " + _CliquesSet[csi].getName());
					_CliquesBaseNodes[csi].add(v);
					csi = max;
				}
			}
		}
		if (_vis) VC().pushAndApplyOperator(new CodePageSelect(-1));
		if(VC()!=null) VC().commitTransaction(); 
	}
}