package comsys.processing.saliency;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Hashtable;
import java.util.Vector;
import java.util.Enumeration;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Set;

public class WeightedWordClasses {

	Hashtable<String,WeightedWordClass> classes;
	
	Hashtable<String,WeightedWordClass> initialClasses;
	
	public WeightedWordClasses(String fileName) {
		Vector<String> lines = new Vector<String>();
		try {
			BufferedReader in = new BufferedReader(new FileReader(fileName));
			String line = in.readLine() ;
			while (line != null) {
				lines.add(line);
				line = in.readLine();
			}
			processLines(lines);
		}
		catch (Exception e) {
			System.out.println("ERROR, file "+ fileName + " specifying the word classes doesnt exist!!");
			e.printStackTrace();
		}
	}
	
	public WeightedWordClasses(Vector<String> lines) {
		processLines(lines);
	}
	
	public void processLines(Vector<String> lines) {
			classes = new Hashtable<String,WeightedWordClass>();
			initialClasses = new Hashtable<String,WeightedWordClass>();
			String lastClass = "";
			for (Enumeration<String> e = lines.elements() ; e.hasMoreElements() ;) {
				String line = e.nextElement() ;
				if (line.contains("[")) {
					String[] header = line.trim().split(" ");
					WeightedWordClass classElements = 
						new WeightedWordClass();
					String className = header[0].replace(":dynaref", "");
					classes.put(className, classElements);
					initialClasses.put(className, classElements);
					lastClass = className;
				}
				else if (line.contains("(") && line.contains(")")) {
					String strippedLine = line.replace("\t", "").replace("(", "").replace(")", "");
					String[] splittedLine = strippedLine.split("~");
					String word = splittedLine[0];
					float weight = 1.0f;
					if (splittedLine.length > 1) {
						weight = new Float(splittedLine[1]).floatValue();
					}
					WeightedWordClass classElements = classes.get(lastClass);
					WeightedWordClass classElementsInit = initialClasses.get(lastClass);
					classElements.addWeightedWord(new WeightedWord(word,weight));
					classElementsInit.addWeightedWord(new WeightedWord(word,weight));
				}
			}
	}
	
	public void reset() {
		classes = new Hashtable<String,WeightedWordClass>();
		for (Enumeration<String> e = initialClasses.keys() ; e.hasMoreElements() ;) {
			String key = e.nextElement() ;
			WeightedWordClass initwclass = initialClasses.get(key);
			WeightedWordClass newClass = new WeightedWordClass();
			for (Enumeration<WeightedWord> f = initwclass.getWeightedWords(); f.hasMoreElements() ;) {
				WeightedWord initwword = f.nextElement() ;
				newClass.addWeightedWord(new WeightedWord(initwword.word, initwword.weight));
			}
			classes.put(key, newClass);
		}
	}
	
	public String toString() {
		String result = "";
		for (Enumeration<String> e = classes.keys() ; e.hasMoreElements() ;) {
			result += toStringOneClass(e.nextElement()) + "\n";
		}
		return result;
	}
	
	public String toStringOneClass(String key) {
		String result = "";
		result += key + ":dynaref [\n";
		WeightedWordClass weightedWords = classes.get(key);
		for (Enumeration<WeightedWord> f = weightedWords.getWeightedWords() ; f.hasMoreElements() ;) {
			WeightedWord wword = f.nextElement(); 
			String weight = new Float(wword.weight).toString();
			if (weight.length() > 5) {
				weight = weight.substring(0, 4);
			}
			result +=  "\t(" + wword.word  + ")~" + weight + "\n";
		}
		result += "]\n\n";
		return result;
	}
	
	public Set<String> getClassNames() {
		return classes.keySet() ;
	}
	
	public WeightedWordClass getClassContent(String className) {
		if (classes.containsKey(className)) {
			return classes.get(className);
		}
		else {
			return null;
		}
	}
	
	public void writeToOutputFile(String fileName) {
		try {
			BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
			out.write(toString());
			out.close();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
	}
	
	public void writeToOutputFileOneClass(String fileName, String specClass) {
		try {
			if (classes.containsKey(specClass)) {
			BufferedWriter out = new BufferedWriter(new FileWriter(fileName));
			out.write(";GSL2.0\n\n" + toStringOneClass(specClass));
			out.close();
			}
			else {
				System.out.println(specClass + " not found");
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
	}
	
}
