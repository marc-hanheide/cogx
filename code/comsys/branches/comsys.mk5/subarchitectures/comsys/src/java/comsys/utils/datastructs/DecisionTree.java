package comsys.utils.datastructs;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;

import comsys.datastructs.comsysEssentials.*;
import comsys.lf.utils.LFUtils;
import java.util.Vector ;
import java.util.Enumeration ;

public class DecisionTree {

	TreeNode root ;

	List<String> decisionsList ;

	public DecisionTree (String[] decisionsList) {
		if (decisionsList.length > 0) {
			this.decisionsList = new ArrayList<String>(Arrays.asList(decisionsList));
			root = new TreeNode(decisionsList[0], TreeNode.TYPE_DISCRIM);
		}
		else {
			log("Warning, decisionList is empty!");
		}
	}

	public Iterator<String> getDecisionsList() {
		return decisionsList.iterator();
	}

	public String getDialogueMove (List<AbstractFeatureValue> avm) {
		List<String> features = new ArrayList<String>();
		List<Object> values = new ArrayList<Object>();
		Iterator<AbstractFeatureValue> it = avm.iterator();
		while (it.hasNext()) {
			AbstractFeatureValue fv = it.next();
			features.add(fv.getFeat());
			values.add(fv.getValue());
		}		
		return getDialogueMoveRecursive(features, values, root);
	}

	public String getDialogueMoveRecursive (List<String> features, 
			List<Object> values, TreeNode curNode) {
		
		String feature = curNode.label.toLowerCase();
		String result = "none";
		if (curNode.type == TreeNode.TYPE_DIALOGUEMOVE) {
			return curNode.label;
		}
		else if (features.contains(feature)) {

			Object abstractValue = (Object) values.get(features.indexOf(feature));
			Vector<String> potentialValues = new Vector<String>();

			if (abstractValue.getClass().getName().equals("java.lang.String")) {
				potentialValues.add((String)abstractValue);
			}
			else if (abstractValue.getClass().getName().equals("java.util.Vector")) {
				potentialValues.addAll((Vector<String>)abstractValue);
			}
			
			for (Enumeration<String> e = potentialValues.elements() ; e.hasMoreElements() ; ) {
				String value = e.nextElement();
				if (curNode.getDaughtersEdgeLabels().contains(value.toLowerCase())) {
					TreeNode daughter= curNode.getDaughter(value.toLowerCase());
					return getDialogueMoveRecursive (features, values, daughter);
				}
				else if (curNode.getDaughtersEdgeLabels().contains("default")) {
					TreeNode daughter = curNode.getDaughter("default");
					return getDialogueMoveRecursive (features, values, daughter);			
				}
				else {
					log("value " + value + " NOT contained in the values set");
				}
			}

		}
		else {
			if (curNode.getDaughtersEdgeLabels().contains("default")) {
				TreeNode daughter = curNode.getDaughter("default");
				return getDialogueMoveRecursive (features, values, daughter);
			}
			else {
				log("feature " + curNode.label + " NOT contained in the features set");
			}
		}

		return result;		
	}

	public void addDialogueMove (String DMName, List<AbstractFeatureValue> avm) {
		List<String> features = new ArrayList<String>();
		List<Object> values = new ArrayList<Object>();
		Iterator<AbstractFeatureValue> it = avm.iterator();
		while (it.hasNext()) {
			AbstractFeatureValue fv = it.next();
			features.add(fv.getFeat());
			values.add(fv.getValue());
		}
		addDialogueMoveRecursive(DMName, features, values, 0, root);
	}

	public void addDialogueMoveRecursive(String DMName, List<String> features, 
			List<Object> values, int curIndex, TreeNode curNode) {

		if (features.size() != values.size()) {
			log("Error: the sizes of the features list and of the feature values list are different");
		}
		else if (curIndex > decisionsList.size() - 1) {
			log("ERROR: current index out of the bounds of the decisions list");
		}
		else {
			String curFeature = decisionsList.get(curIndex);
			if (features.contains(curFeature.toLowerCase())) {
				
				Object abstractValue = values.get(features.indexOf(curFeature.toLowerCase()));
				Vector<String> potentialValues = new Vector<String>();

				if (abstractValue.getClass().getName().equals("java.lang.String")) {
					potentialValues.add((String)abstractValue);
				}
				else if (abstractValue.getClass().getName().equals("java.util.Vector")) {
					potentialValues.addAll((Vector<String>)abstractValue);
				}
				else {
				}
				
				for (Enumeration<String> e = potentialValues.elements() ; e.hasMoreElements() ; ) {
					String value = e.nextElement();
				if (curNode.hasDaughter(value.toLowerCase())) {
					TreeNode daughter = curNode.getDaughter(value);
					if (daughter != null) {
						curIndex++;
						addDialogueMoveRecursive(DMName, features, values, curIndex, daughter);
					}
					else {
						log("Error, daughter of "+ curNode.label + " is a null element");
					}
				}
				else {
					curIndex++;
					if (curIndex < decisionsList.size()) {
						String nodeLabel = decisionsList.get(curIndex);
						TreeNode daughter = new TreeNode(nodeLabel, TreeNode.TYPE_DISCRIM);
						TreeEdge edge = new TreeEdge(curNode, daughter, value);
						curNode.addDaughter(edge);
						addDialogueMoveRecursive(DMName, features, values, curIndex, daughter);
					}
					else {
						TreeNode daughter = new TreeNode(DMName, TreeNode.TYPE_DIALOGUEMOVE);
						TreeEdge edge = new TreeEdge(curNode, daughter, value);
						curNode.addDaughter(edge);				
					}
				}
				
				}
			}
			else {
				curIndex++;
				if (curIndex < decisionsList.size()) {
					if (!curNode.hasDaughter("default")) {
						String nodeLabel = decisionsList.get(curIndex);
						TreeNode daughter = new TreeNode(nodeLabel, TreeNode.TYPE_DISCRIM);
						TreeEdge edge = new TreeEdge(curNode, daughter, "default");
						curNode.addDaughter(edge);
						addDialogueMoveRecursive(DMName, features, values, curIndex, daughter);
					}
					else {
						TreeNode daughter = curNode.getDaughter("default");
						if (daughter != null) {
							addDialogueMoveRecursive(DMName, features, values, curIndex, daughter);					
						}
					}
				}
				else {
					TreeNode daughter = new TreeNode(DMName, TreeNode.TYPE_DIALOGUEMOVE);
					TreeEdge edge = new TreeEdge(curNode, daughter, "default");
					curNode.addDaughter(edge);				
				}

			}
		}

	}


	public String createDOTSpecs() {
		String text = "digraph G {\n";
		if (root != null) {
			text += root.createDOTSpecs() ;
		}
		text = text + "\n}";
		return text;
	}

	public void DecisionTreeToGraph(String graphName) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecs() ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		}
	}


	public void log(String message) {
		System.out.println("[DecisionTree] " + message);
	}

}

