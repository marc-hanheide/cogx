package binder.gui;

import java.awt.Dimension;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

import javax.swing.JFrame;

import org.apache.commons.collections15.Transformer;

import cast.cdl.WorkingMemoryAddress;
import beliefmodels.autogen.beliefs.AttributedBelief;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.history.BeliefHistory;
import beliefmodels.autogen.history.CASTBeliefHistory;
import edu.uci.ics.jung.algorithms.layout.AbstractLayout;
import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.algorithms.layout.FRLayout2;
import edu.uci.ics.jung.algorithms.layout.Layout;
import edu.uci.ics.jung.graph.DirectedSparseGraph;
import edu.uci.ics.jung.graph.DirectedSparseMultigraph;
import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.event.GraphEvent.Vertex;
import edu.uci.ics.jung.graph.util.EdgeType;
import edu.uci.ics.jung.visualization.VisualizationViewer;



public class BeliefGraph {
	
//	private BeliefNode node;
//	private BeliefEdge edge;
//	private Belief belief;
	
	private static HashMap<WorkingMemoryAddress,Belief > _beliefMap = new HashMap<WorkingMemoryAddress,Belief >();
	
	private  Graph <BeliefNode,BeliefEdge> g = new DirectedSparseGraph<BeliefNode,BeliefEdge>();
	
	private static HashMap<String,BeliefNode> _beliefNodeMap = new HashMap<String, BeliefNode>();

	private static LinkedList<Graph<BeliefNode,BeliefEdge>> graphList = new LinkedList<Graph<BeliefNode,BeliefEdge>>();
	
	private BeliefGraphViewer viewer = new BeliefGraphViewer();
	
	private AbstractLayout<BeliefNode, BeliefEdge> layout;
	 private VisualizationViewer<BeliefNode, BeliefEdge> vv;
	

	
	public  void  createGraph (Belief belief,WorkingMemoryAddress wmc){
		
			
			//map each belief to workingmemoryaddress
			_beliefMap.put(wmc, belief);	
			
			//create Vertices from Beliefs now
			BeliefNode beliefNode = new BeliefNode(belief,belief.id);
			_beliefNodeMap.put(belief.id,beliefNode);
			
//			//specify the layout to be used
//			Layout<BeliefNode,BeliefEdge> layout = new CircleLayout<BeliefNode,BeliefEdge>(g);
//			
//			
//			//specify the renderer
//			//Renderer<BeliefNode,BeliefEdge> renderer = new BasicRenderer<BeliefNode,BeliefEdge>();
//			
//			VisualizationViewer<BeliefNode, BeliefEdge> vv = new VisualizationViewer<BeliefNode, BeliefEdge>(layout);
			
			
			g.addVertex(beliefNode);
			
			//now create the edges
			if (belief.hist !=null){
				if (new CASTBeliefHistory() instanceof BeliefHistory) {
					CASTBeliefHistory beliefhistory = (CASTBeliefHistory) belief.hist;
					Iterator<WorkingMemoryAddress> ancestors = beliefhistory.ancestors.iterator();
					
					if (ancestors.hasNext()){
						WorkingMemoryAddress wma = ancestors.next();
						Belief ancestorbelief = _beliefMap.get(wma);
						
							if (ancestorbelief !=null){
								BeliefNode ancestorNode = new BeliefNode(ancestorbelief,ancestorbelief.id);
								g.addEdge(new BeliefEdge(""),beliefNode,ancestorNode , EdgeType.DIRECTED); // This method		
							}
						
						}
						
					
					}	
				}
			
			
			
			//graphList.add(g);
			
		

	
	}
	
	public void init(){
		
		layout = new FRLayout2<BeliefNode, BeliefEdge>(g);
	    vv = new VisualizationViewer<BeliefNode,BeliefEdge>(layout, new Dimension(600,600));
	     
	 	
			vv.setPreferredSize(new Dimension(350,350)); //Sets the viewing area size
			
			// Setup up a new vertex to text transformer...
			Transformer<BeliefNode, String> vertexPaint = new Transformer<BeliefNode,String>() {
				
				@Override
				public String transform(BeliefNode arg0) {
					// TODO Auto-generated method stub
					return arg0.beliefid;
				}
				};
				
			//vertex tool tip
				
			Transformer<BeliefNode, String>	vertexInfo = new Transformer<BeliefNode, String>() {

				@Override
				public String transform(BeliefNode arg0) {
					// TODO Auto-generated method stub
					String type = new String();
					if (arg0.belief instanceof PerceptBelief)
						type = "PerceptBelief";
					if (arg0.belief instanceof PerceptUnionBelief)
						type = "PerceptUnionBelief";
					if (arg0.belief instanceof MultiModalBelief)
						type = "MultiModalBelief";
					if (arg0.belief instanceof TemporalUnionBelief)
						type = "TemporalUnionBelief";
					if (arg0.belief instanceof StableBelief)
						type = "StableBelief";
					if (arg0.belief instanceof AttributedBelief){
						type = "Attributed Belief";
					}
							
					return type;
				}
			};
			
			
			Transformer<BeliefNode, String>	vertexInfo1 = new Transformer<BeliefNode, String>() {

				@Override
				public String transform(BeliefNode arg0) {
					// TODO Auto-generated method stub
					String type = new String();
					if (arg0.belief instanceof PerceptBelief)
						type = arg0.belief.type;
					if (arg0.belief instanceof PerceptUnionBelief)
						type = arg0.belief.hist.toString();

					if (arg0.belief instanceof MultiModalBelief)
						type = arg0.belief.type;
					if (arg0.belief instanceof TemporalUnionBelief)
						type = arg0.belief.type;
					if (arg0.belief instanceof StableBelief)
						type = arg0.belief.type;
					if (arg0.belief instanceof AttributedBelief){
						type = arg0.belief.type;
					}
							
					return type;
				}
			};
			
			
			//vv.getRenderContext().setVertexDrawPaintTransformer(vertexPaint);
			vv.getRenderContext().setVertexLabelTransformer(vertexPaint);
			vv.setVertexToolTipTransformer(vertexInfo);
		//	vv.setVertexToolTipTransformer(vertexInfo1);

			
			
			

		
	}
	
	
	/*
	 * this method is called when a belief is deleted from the hierarchy
	 */
	public  void deleteGraph(Belief belief ) {
		
		if (_beliefNodeMap.containsKey(belief.id)){
			BeliefNode bNode = _beliefNodeMap.get(belief.id);
			
			LinkedList<BeliefEdge> outEdges= (LinkedList<BeliefEdge>) g.getOutEdges(bNode); 
			
			Iterator<BeliefEdge> edgeItr = outEdges.iterator();
			if (edgeItr.hasNext()){
				BeliefEdge outEdge = edgeItr.next();
				g.removeEdge(outEdge);
				
			}
			
			LinkedList<BeliefEdge> inEdges= (LinkedList<BeliefEdge>) g.getInEdges(bNode); 
			
			Iterator<BeliefEdge> eItr = inEdges.iterator();
			if (eItr.hasNext()){
				BeliefEdge inEdge = eItr.next();
				g.removeEdge(inEdge);	
			}	
			g.removeVertex(bNode);
		}		
	}
	
	public  void display(){
		//BeliefGraphViewer.displayBeliefGraph(graphList);
		
		JFrame frame = new JFrame("Belief Graph Viewer");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().add(vv);
		
		frame.pack();
		frame.setVisible(true);
	}
	

}
