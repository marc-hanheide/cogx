package comsys.utils.datastructs;

import java.util.Vector;
import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;

public class TreeNode {
	
	public static final int TYPE_DISCRIM = 0;
	public static final int TYPE_DIALOGUEMOVE = 1;
	
	String label = "";
	TreeEdge mother ;
	Vector<TreeEdge> daughters = new Vector<TreeEdge>();
	int type;

	int identifier;
	static int identifierIncr;
	
	public TreeNode (String label, int type) {
		this.label = label ;
		if (type != TYPE_DISCRIM && type != TYPE_DIALOGUEMOVE) {
			log("ERROR: node type is not valid");
		}
		else {
			this.type = type;
		}
		identifier = identifierIncr;
		identifierIncr++;
	}
	
	public void setLabel(String label) {
		this.label = label ;
	}
	
	public void setMother (TreeEdge mother) {
		this.mother = mother;
	}
	
	public void setMother (TreeNode mother, String edgeLabel) {
		TreeEdge treeEdge = new TreeEdge(mother, this, edgeLabel);
		this.mother = treeEdge;
	}
	
	public void addDaughter(TreeEdge treeEdge) {
		daughters.add(treeEdge);
	}
	
	public void addDaugher(TreeNode daughter, String edgeLabel) {
		TreeEdge treeEdge = new TreeEdge(this, daughter, edgeLabel);
		daughters.add(treeEdge);
	}
	
	
	public String getLabel() {
		return label;
	}
	
	public boolean isRoot() {
		return (mother==null);
	}
	
	public TreeEdge getMother() {
		return mother;
	}
	
	public Iterator<TreeEdge> getDaughters() {
		return daughters.iterator();
	}
	
	public TreeNode getDaughter (String edgeLabel) {
		Iterator<TreeEdge> it = getDaughters();
		while (it.hasNext()) {
			TreeEdge treeEdge = it.next();
			if (treeEdge.getEdgeLabel().equals(edgeLabel)) {
				return treeEdge.daughter;
			}
		}
		return null;
	}
	
	public List<String> getDaughtersEdgeLabels() {
		List<String> labels = new ArrayList<String>();
		Iterator<TreeEdge> it = getDaughters();
		while (it.hasNext()) {
			labels.add(it.next().edgeLabel);
		}
		return labels;
	}
	
	public boolean hasDaughter(String edgeLabel) {
		Iterator<TreeEdge> it = getDaughters();
		while (it.hasNext()) {
			TreeEdge treeEdge = it.next();
			if (treeEdge.getEdgeLabel().equals(edgeLabel)) {
				return true;
			}
		}
		return false;
	}
	
	public int getIdentifier() { return identifier; }
	
	public int getType() {
		return type;
	}

	public String createDOTSpecs() {
		String text = label.replace('-','_') + identifier + " [label=\""+label+"\"];\n";
		Iterator<TreeEdge> it = getDaughters();
		while (it.hasNext()) {
			TreeEdge te = it.next();
			if (te.mother != null && te.daughter != null && te.edgeLabel != null) {
				String motherLabel = te.mother.label.replace('-','_') + te.mother.identifier;
				String edgeLabel = te.edgeLabel.replace("+", "pos").replace("-","neg");
				TreeNode curDaughter = te.daughter;
			//	while (curDaughter.daughters.size() == 1 && curDaughter.daughters.elementAt(0).edgeLabel.equals("default")) {
			//		curDaughter = curDaughter.daughters.elementAt(0).daughter;
			//	}
				String daughterLabel = curDaughter.label.replace('-','_') + curDaughter.identifier;
				text += motherLabel + " -> " + daughterLabel + " [label=\""+edgeLabel+"\"];\n";
				text += curDaughter.createDOTSpecs();
			}
		}
		return text;
	}

	public void log(String message) {
		System.out.println("[TreeNode] " + message);
	}
}
