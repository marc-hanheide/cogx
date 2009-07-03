package comsys.processing.saliency;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

public class WordsActivationNetwork {

	Hashtable<String, WeightedWordClasses> activations;

	public WordsActivationNetwork(String fileName) {
		activations = new Hashtable<String,WeightedWordClasses>();
		try {
			BufferedReader in = new BufferedReader(new FileReader(fileName));
			String line = in.readLine() ;
			String concept = "";
			Vector<String> lines = new Vector<String>();
			while (line != null) {
				if (line.contains("{")) {
					String[] header = line.trim().split(" ");
					concept = header[0];
					lines = new Vector<String>();
				}
				else if (line.contains("}")) {
					WeightedWordClasses classes = new WeightedWordClasses(lines); 
					activations.put(concept, classes);
				}
				else {
					lines.add(line);
				}
				line = in.readLine();
			}
		}
		catch (Exception e) {
			System.out.println("ERROR, file "+ fileName + " specifying the word activation network doesnt exist!!");
			e.printStackTrace();
		}
	}
	
	public WeightedWordClasses getActivation(String entity) {
		return activations.get(entity);
	}
	
	public String toString() {
		String result = "";
		for (Enumeration<String> e = activations.keys() ; e.hasMoreElements() ;) {
			String key = e.nextElement();
			result += key + "{\n";
			result += activations.get(key).toString();
			result += "}\n\n";
		}
		return result;
	}
	
}
	

