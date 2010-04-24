package binder.gui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Paint;

import javax.swing.JFrame;
import javax.swing.JRootPane;

import org.apache.commons.collections15.Transformer;

import beliefmodels.autogen.beliefs.AttributedBelief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;

import edu.uci.ics.jung.algorithms.layout.CircleLayout;
import edu.uci.ics.jung.algorithms.layout.FRLayout2;
import edu.uci.ics.jung.algorithms.layout.Layout;
import edu.uci.ics.jung.graph.DirectedSparseMultigraph;
import edu.uci.ics.jung.graph.Graph;
import edu.uci.ics.jung.graph.ObservableGraph;
import edu.uci.ics.jung.graph.event.GraphEvent;
import edu.uci.ics.jung.graph.event.GraphEventListener;
import edu.uci.ics.jung.graph.util.Graphs;
import edu.uci.ics.jung.visualization.BasicVisualizationServer;
import edu.uci.ics.jung.visualization.VisualizationViewer;
import edu.uci.ics.jung.visualization.control.DefaultModalGraphMouse;
import edu.uci.ics.jung.visualization.decorators.ToStringLabeller;
import edu.uci.ics.jung.visualization.renderers.Renderer;



public class BeliefGraphViewer {
	


	public  void displayBeliefGraph(Graph<BeliefNode,BeliefEdge> g){
	
	
		
		//specify the layout to be used
		Layout<BeliefNode,BeliefEdge> layout = new CircleLayout<BeliefNode,BeliefEdge>(g);
		
		
		//specify the renderer
		//Renderer<BeliefNode,BeliefEdge> renderer = new BasicRenderer<BeliefNode,BeliefEdge>();
		
		VisualizationViewer<BeliefNode, BeliefEdge> vv = new VisualizationViewer<BeliefNode, BeliefEdge>(layout);
//		BasicVisualizationServer<BeliefNode,BeliefEdge> vv =
//			new BasicVisualizationServer<BeliefNode,BeliefEdge>(layout);
		
		//GraphChangeListener l = new GraphChangeListener();
		
	//	vv.addChangeListener(l);
		
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

		
		JFrame frame = new JFrame("Belief Graph Viewer");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().add(vv);
		
		frame.pack();
		frame.setVisible(true);
		

	}
	
	public void refreshBeliefGraph(Graph<BeliefNode,BeliefEdge> g){
		
        //create a graph
//		Graph<Number,Number> ig = Graphs.<Number,Number>synchronizedDirectedGraph(new DirectedSparseMultigraph<Number,Number>());
//    	Graph<BeliefNode,BeliefEdge> bg = Graphs.synchronizedDirectedGraph(new DirectedSparseMultigraph<BeliefNode, BeliefEdge>());
		
		ObservableGraph<BeliefNode,BeliefEdge> og = new ObservableGraph<BeliefNode,BeliefEdge>(g);
        og.addGraphEventListener(new GraphEventListener<BeliefNode,BeliefEdge>() {

			public void handleGraphEvent(GraphEvent<BeliefNode, BeliefEdge> evt) {
				System.err.println("got "+evt);

			}});
		
		Layout<BeliefNode, BeliefEdge> layout = new FRLayout2<BeliefNode, BeliefEdge>(g);
		
//      ((FRLayout)layout).setMaxIterations(200);

     VisualizationViewer<BeliefNode, BeliefEdge> vv = new VisualizationViewer<BeliefNode,BeliefEdge>(layout, new Dimension(600,600));

      //JRootPane rp = this.getRootPane();
//      rp.putClientProperty("defeatSystemEventQueueCheck", Boolean.TRUE);
//
//      getContentPane().setLayout(new BorderLayout());
//      getContentPane().setBackground(java.awt.Color.lightGray);
//      getContentPane().setFont(new Font("Serif", Font.PLAIN, 12));

      //vv.getModel().getRelaxer().setSleepTime(500);
      vv.setGraphMouse(new DefaultModalGraphMouse<Number,Number>());

      vv.getRenderer().getVertexLabelRenderer().setPosition(Renderer.VertexLabel.Position.CNTR);
     // vv.getRenderContext().setVertexLabelTransformer(new ToStringLabeller<Number>());
      vv.setForeground(Color.white);
     // getContentPane().add(vv);
		
	}
	
	
}
