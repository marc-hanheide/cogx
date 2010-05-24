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
import edu.ksu.cis.util.graph.visualization.*;
import edu.ksu.cis.util.graph.visualization.operators.Delay;
/**
 * file: Triangulation.java
 * 
 * @author Jeffrey M. Barber
 */
public class TriangulationByMaxCardinalitySearch extends Algorithm
{
	private int[]		alpha		= null;
	private Vertex[]	alphainv	= null;
	public TriangulationByMaxCardinalitySearch()
	{_Name = "Triangulation By Max Cardinality Search";	}
	
	
	/**
	 * @return
	 */
	public int[] getAlpha()
	{
		return alpha;
	}
	/**
	 * @return
	 */
	public Vertex[] getAlphaInverse()
	{
		return alphainv;
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see edu.ksu.cis.util.graph.visualization.Algorithm#execute()
	 */
	public void execute()
	{
		
		boolean _vis = canVisualize();
		if (_vis) VC().beginTransaction();
		if (_vis) VC().pushAndApplyOperator(new Annotation("Triangulation By Max Cardinality Search"));
		if (_vis) VC().pushAndApplyOperator(new Delay("delay_vis_triangulation_by_mcs",1000));
		
		RemoveDirectionality RD = new RemoveDirectionality();
		Moralization Moral = new Moralization();
		MaximumCardinalitySearch MCS = new MaximumCardinalitySearch();
		// Execute Moralization
		Moral.setGraph(_Graph);
		Moral.setVisualization(VC());
		Moral.execute();
		_Graph = Moral.getGraph();
		// Remove Directionality
		RD.setGraph(_Graph);
		RD.setVisualization(VC());
		RD.execute();
		_Graph = RD.getGraph();
		// Maximum Cardinality Search
		MCS.setGraph(_Graph);
		MCS.setVisualization(VC());
		MCS.execute();
		_Graph = MCS.getGraph();
		
		alpha = MCS.getAlpha();
		alphainv = MCS.getAlphaInverse();
		// Fill-In Computation
		FillIn FIN = new FillIn(alpha, alphainv);
		FIN.setGraph(_Graph);
		FIN.setVisualization(VC());
		FIN.execute();
		_Graph = FIN.getGraph();
		if (_vis) VC().commitTransaction();
	}
}