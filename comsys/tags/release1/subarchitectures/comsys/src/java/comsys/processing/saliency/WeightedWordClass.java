package comsys.processing.saliency;

import java.util.Vector;
import java.util.Hashtable;
import java.util.Enumeration;
import java.util.Iterator;

public class WeightedWordClass {

	Hashtable<String, WeightedWord> weightedWords ;
	
	public WeightedWordClass() {
		weightedWords = new Hashtable<String,WeightedWord>();
	}
	
	public WeightedWordClass(Vector<WeightedWord> wwords) {
		this();
		for (Iterator<WeightedWord> e = wwords.iterator() ; e.hasNext() ;) {
			WeightedWord wword = e.next();
			weightedWords.put(wword.word, wword);
		}
	}
	
	public void addWeightedWord(WeightedWord wword) {
		weightedWords.put(wword.word, wword);
	}
	
	public WeightedWord getWeightedWord(String str) {
		if (weightedWords.containsKey(str)) {
			return weightedWords.get(str);
		}
		else {
			return null;
		}
	}
	
	public int size()  {
		return weightedWords.size() ;
	}
	
	public Enumeration<WeightedWord> getWeightedWords() {
		return weightedWords.elements() ;
	}
	
	public boolean hasWeightedWord(String str) {
		return weightedWords.containsKey(str);
	}
	
	public String toString() {
		String result = "";
		for (Enumeration<WeightedWord>  e = weightedWords.elements() ; e.hasMoreElements() ;) {
			WeightedWord word = e.nextElement();
			result += word.word + "~" + word.weight + "\n";
		}
		return result;
	}
}
