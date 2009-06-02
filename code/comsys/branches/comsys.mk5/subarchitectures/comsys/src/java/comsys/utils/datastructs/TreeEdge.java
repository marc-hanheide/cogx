package comsys.utils.datastructs;

public class TreeEdge {

	String edgeLabel ;
	TreeNode mother ;
	TreeNode daughter ;
	
	public TreeEdge (TreeNode mother, TreeNode daughter, String edgeLabel) {
		if (mother != null && daughter != null & edgeLabel != null) {
		this.mother = mother;
		this.daughter = daughter;
		this.edgeLabel = edgeLabel ;
		}
		else {
			log("Error: trying to create a tree edge with null elements");
		}
	}
	
	public TreeNode getMother() { return mother; }
	
	public TreeNode getDaughter() { return daughter ; }
	
	public String getEdgeLabel() { return edgeLabel; }
	

	public void log(String message) {
		System.out.println("[TreeEdge] " + message);
	}
}
