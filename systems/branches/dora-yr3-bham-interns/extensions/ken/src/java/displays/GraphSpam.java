package displays;

import clustering.ProbGenerator;

public class GraphSpam {

	public static void main(String[] args) {
		
			//GraphSpam g = new GraphSpam(0, false, 1);
			//PathVisualization.value=PathVisualization.values[1];
			GraphSpam g3 = new GraphSpam(69,false, 1);
			
	
//		for(int i=0;i<69;i++){
//			GraphSpam g = new GraphSpam(i, false, 1);
//		}
		

	}

	public GraphSpam(int n, boolean day, int dayToLookAt) {

		//PathVisualization.value = PathVisualization.values[0];

		PathVisualization c = new PathVisualization(n, day);
		ProbGenerator p = new ProbGenerator(n, true,false, day, dayToLookAt);
//
		double[] e = p.getEntropies();
		System.out.println("entropy");
		EntropyVis eVis = new EntropyVis(e, 40, 80, 10, false,
				"Entropies Between Nodes " + c.start + " & " + c.end);
		int[] est = p.getEstimates();
		BelievedTravelVis tVis = new BelievedTravelVis(est, 40, 80, 10, false,
				"Travel Times Between Nodes " + c.start + " & " + c.end);

	}

}
