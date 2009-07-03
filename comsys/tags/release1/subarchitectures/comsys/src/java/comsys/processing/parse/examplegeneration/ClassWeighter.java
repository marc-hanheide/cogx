
package comsys.processing.parse.examplegeneration;

import java.io.BufferedReader;
import java.io.StringReader;
import java.util.Hashtable;
import java.util.Vector;
import java.util.Enumeration;


public class ClassWeighter {
	
	static boolean useSmoothing = true;

	public static String weightClasses(String inputClasses, String inputRawCorpus) {
	String result = "";
		try {
		BufferedReader in = new BufferedReader(new StringReader(inputClasses));
		String line = in.readLine() ;
		Hashtable<String,Vector<String>> classes = new Hashtable<String,Vector<String>>();
		String lastClass = "";
		while (line != null) {
			if (line.contains("[")) {
				String[] header = line.split(" ");
				//	log("HEADER: " + header[0]);
				Vector<String> classElements = new Vector<String>();
				classes.put(header[0], classElements);
				lastClass = header[0];
			}
			else if (line.contains("(") && line.contains(")")) {
				String element = line.replace("\t", "").replace("(", "").replace(")", "");
				//	log("ELEMENT: " + element + " in " + lastClass);
				Vector<String> classElements = classes.get(lastClass);
				classElements.add(element);
			}
			line = in.readLine();
		}
		//	log(classes.toString());
		Hashtable<String, Hashtable<String, Float>> allCounts = 
			new Hashtable<String, Hashtable<String, Float>>();
		BufferedReader corpus = new BufferedReader(new StringReader(inputRawCorpus));
		
		// loop on classes

		for (Enumeration<String> it = classes.keys() ; it.hasMoreElements() ; ) {
			String class1 = it.nextElement();

			Hashtable<String, Float> counts = new Hashtable<String, Float>();
			allCounts.put(class1, counts);
			Vector<String> classElements = classes.get(class1);
			int totalClassCount = 0;

			// loop on class elements
			for (Enumeration<String> it2 = classElements.elements() ; it2.hasMoreElements() ;) {
				String element = it2.nextElement();

				String linec = corpus.readLine();
				while (linec!= null) {
					if (linec.contains(element)) {
						totalClassCount++;
						if (counts.containsKey(element)) {
							Float newCount = new Float(counts.get(element).intValue() + 1.0);
							counts.put(element, newCount);
						}
						else {
							counts.put(element, new Float(1.0));
						}
			  		}
					linec = corpus.readLine();
				}
				corpus.close();
				corpus = new BufferedReader(new StringReader(inputRawCorpus));
			} 
     
			/**
			for (Enumeration<String> it2 = classElements.elements() ; it2.hasMoreElements() ;) {
				String element = it2.nextElement();
				float probability;
				if (counts.containsKey(element)) {
					probability = (counts.get(element)+5.0f)/(totalClassCount + 5*classElements.size());
				}
				else {
					probability = 5.0f/ (totalClassCount + 5*classElements.size());
				}
				out.write(class1 + " " + probability +" " + element + "\n");
			}*/

			
			result += class1 + ":dynaref [\n";
			for (Enumeration<String> it2 = classElements.elements() ; it2.hasMoreElements() ;) {
				String element = it2.nextElement();
				if (counts.containsKey(element)) {
					float weight = (counts.get(element)/ totalClassCount) * classElements.size() ;
					float smoothedWeight = (weight + 1)/2 ;
					smoothedWeight = (smoothedWeight + 1)/2 ;
					float finalWeight = weight;
					if (useSmoothing) {
						finalWeight = smoothedWeight;
					}
					counts.put(element, new Float(finalWeight));
				}
				else {
					counts.put(element, new Float("0.8"));
				}
				if (counts.get(element).toString().length() > 5) {
					String strFloat = counts.get(element).toString().substring(0, 5);
					result += "\t(" + element + ")~" + strFloat + "\n";
				}
				else {
					result += "\t(" + element + ")~" + counts.get(element).toString() + "\n";
				}
				
			}
			result += " ]\n\n"; 
		}
	} 
	catch (Exception e) {
		e.printStackTrace();
	}
	return result;
	}
	
	public static void setSmoothing(boolean smoothing) {
		useSmoothing = smoothing;
	}
	
	private static void log(String s) {
		System.out.println("[CLASS WEIGHTER] " +s) ;
	}

	public static void main(String[] args) {
		
		weightClasses("..//data//classes.grammar", "..//data//fullcorpus.txt");

	}
}
