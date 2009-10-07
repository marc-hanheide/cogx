
package comsys.processing.parse.examplegeneration;

import java.util.*;

/**
 * Context-free grammar and related utilities
 * 
 * @author plison
 *
 */
public class CFG {

	/** the start symbol */
	String StartSymbol;

	/** the context-free rules, indexed by start symbol */
	Hashtable<String,Vector<Rule>> Rules ;

	/** whether to modify the rules weights during recognition */
	boolean weightCountActivated = false;
	
	public static boolean isClassBased = false;

	/**
	 * Constructor
	 *
	 */
	public CFG () {
		Rules = new Hashtable<String,Vector<Rule>>();
	}


	/**
	 * Set the CFG start symbol
	 * @param symbol
	 */
	public void setStartSymbol(String symbol) {
		StartSymbol = symbol;
	}

	/**
	 * Add a new context-free rule, composed of a left-hand side and a right-hand side
	 * @param LHS
	 * @param RHS
	 */
	public void addRule(String LHS, Vector<String> RHS) {
		//	log("Add rule: " + LHS + " ==> " +  RHS.toString());
		if (Rules.containsKey(LHS)) {
			Vector<Rule> curRHSs = Rules.get(LHS);
			curRHSs.add(new Rule(LHS, RHS));
			Rules.put(LHS, curRHSs);
		}
		else {
			Vector<Rule> newRHSs = new Vector<Rule>();
			newRHSs.add(new Rule(LHS, RHS));
			Rules.put(LHS, newRHSs);
		}
	}

	/**
	 * Add a new context-free rule
	 * @param rule
	 */
	public void addRule(Rule rule) {
		//	log("Add rule: " + rule.LHS + " ==> " +  rule.RHS.toString());
		if (Rules.containsKey(rule.LHS)) {
			Vector<Rule> curRHSs = Rules.get(rule.LHS);
			curRHSs.add(rule);
			Rules.put(rule.LHS, curRHSs);
		}
		else {
			Vector<Rule> newRHSs = new Vector<Rule>();
			newRHSs.add(rule);
			Rules.put(rule.LHS, newRHSs);
		}
	}

	public void addRules(Hashtable<String,Vector<Rule>> rules) {
		for (Enumeration<String> e = rules.keys() ; e.hasMoreElements() ; ) {
			String LHS = e.nextElement();
			for (Enumeration<Rule> f = rules.get(LHS).elements() ; f.hasMoreElements() ;) {
				addRule(f.nextElement());
			}
		}
	}

	public void addRules(String LHS,  Vector<Vector<String>> RHSs) {
		for (Enumeration<Vector<String>> e = RHSs.elements() ; e.hasMoreElements() ; ) {
			addRule(LHS, e.nextElement());
		}
	}

	public void addRules2(String LHS, Vector<Rule> rules) {
		for (Enumeration<Rule> e = rules.elements() ; e.hasMoreElements() ; ) {
			Rule RHS = e.nextElement();
			RHS.setLHS(LHS);
			addRule(RHS);
		}
	}


	public String getStartSymbol()  {
		return StartSymbol;
	}

	public Hashtable<String,Vector<Rule>> getRules() {
		return Rules;
	}

	public String toString() {
		String result = "CFG grammmar:\n";
		for (Enumeration<String> e = Rules.keys() ; e.hasMoreElements() ; ) {
			String key = e.nextElement();
			for (Enumeration<Rule> f= Rules.get(key).elements() ; f.hasMoreElements() ;) {
				Rule rule = f.nextElement();
				result += rule.toString() + "\n";
			}
		}
		return result;
	}


	public boolean recognize(String str) {
		Vector<String> earleyRHS = new Vector<String>();
		if (StartSymbol == null) {
			log("No start symbol defined, exiting");
			return false;
		}
		earleyRHS.add(StartSymbol);
		Rule earleyStartrule = new Rule("earleyStart", earleyRHS);
		EarleyState earleyStartState = new EarleyState (earleyStartrule, 0, 0, 0);

		Chart chart = new Chart();
		chart.enqueueState(earleyStartState, 0);

		String[] words = str.split(" ");

		chart = retrieveChart(words, chart);

		EarleyState successfullEndState = new EarleyState(earleyStartrule, 1, 0, words.length);

		EarleyState successfulParse = chart.getSimilarState(words.length, successfullEndState);

		if (successfulParse != null) {
			//	log("successful parse");
			Vector<EarleyState> pointers = chart.getStates(successfulParse.pointers);	

			while (pointers.size() != 0) {
				EarleyState state = pointers.remove(pointers.size() -1);
				//	log(state.toString());
				pointers.addAll(chart.getStates(state.pointers));
			}
		}


		return successfulParse != null;

	}
	
	
	public String getSemantics(String str) {
		
		EarleyState.alreadyExpanded = "";
		
		Vector<String> earleyRHS = new Vector<String>();
		if (StartSymbol == null) {
			log("No start symbol defined, exiting");
			return "";
		}
		earleyRHS.add(StartSymbol);
		Rule earleyStartrule = new Rule("earleyStart", earleyRHS);
		EarleyState earleyStartState = new EarleyState (earleyStartrule, 0, 0, 0);

		Chart chart = new Chart();
		chart.enqueueState(earleyStartState, 0);

		String[] words = str.split(" ");

		chart = retrieveChart(words, chart);

		EarleyState successfullEndState = new EarleyState(earleyStartrule, 1, 0, words.length);

		EarleyState successfulParse = chart.getSimilarState(words.length, successfullEndState);

		if (successfulParse != null) {
			//	log("successful parse");
			Vector<EarleyState> pointers = chart.getStates(successfulParse.pointers);	

			while (pointers.size() != 0) {
				EarleyState state = pointers.remove(pointers.size() -1);
			//		log(state.toString());
				pointers.addAll(chart.getStates(state.pointers));
			}
		}
		else {
		//	log("NO successful parse found");
		}

		if (successfulParse != null) {
			String result = successfulParse.getSemantics(chart);
			result = result.replaceFirst(" \\^ ", "(");
			result = "@" + result + ")";
			return result;
		}
		
		return "";
	}

	public Chart retrieveChart(String[] words, Chart chart) {


		//	log("utterance: " +str);
		for (int i = 0 ; i <= words.length ; i++) {
			for (Enumeration<EarleyState> e = chart.getChartContent(i).elements(); e.hasMoreElements();) {
				// log("size chart: " + chart.getChartContent(i).size());
				EarleyState state = e.nextElement();
				//		log("state analysed in main loop: " + state.toString());
				if (!state.isComplete() && state.isnextConstituentNonTerminal()) {
					//		log("predictor");
					chart = predictor(state, chart);
				}
				else if (!state.isComplete() && i < words.length && ! state.isnextConstituentNonTerminal()) {
					//		log("scanner");
					chart = scanner(state,chart, words[i]);
				}
				else if (state.isComplete()){
					//		log("completer");
					//		log(state.toString());
					chart = completer(state, chart);
				}
			}		
		}

		return chart;
	}

	private Chart predictor(EarleyState state, Chart chart) {
		String nextConstituent = state.getNextConstituent();
		//	log("Predictor - state: " + state);
		//	log("next constituent: " + nextConstituent);
		if (Rules.containsKey(nextConstituent)) {
			for (Enumeration<Rule> e = Rules.get(nextConstituent).elements() ; e.hasMoreElements() ; ) {
				EarleyState newState = new EarleyState (e.nextElement(), 0, state.chartEnd, state.chartEnd);
				chart.enqueueState(newState, state.chartEnd);
			}
		}
		return chart;
	}

	private Chart scanner(EarleyState state, Chart chart, String word) {
		//	log("scanner: word " + word);
		if(state.getNextConstituent().equals(word)) {
			EarleyState newState = new EarleyState(state.rule, state.posInRule + 1, 
					state.chartStart, state.chartEnd + 1);
			//		log("old state: " + state.toString());
			//		log("new state enqueued: " + newState.toString());
			newState.addPointers(state.pointers);
			chart.enqueueState(newState, state.chartEnd+1);
		}
		return chart;
	}

	private Chart completer(EarleyState state, Chart chart) {
		Vector<EarleyState> chartAtPosJ = chart.getChartContent(state.chartStart);
		for (Enumeration<EarleyState> e = chartAtPosJ.elements() ; e.hasMoreElements() ; ) {
			EarleyState potState = e.nextElement();
			if (potState.getNextConstituent() != null && 
					potState.getNextConstituent().equals(state.rule.LHS) && 
					potState.chartEnd == state.chartStart) {
				EarleyState newState = new EarleyState(potState.rule, 
						potState.posInRule + 1, potState.chartStart, state.chartEnd);
				newState.addPointers(potState.pointers);
				newState.addPointer(state.stateId);
				chart.enqueueState(newState, state.chartEnd);

				if (weightCountActivated) {
					if (state.rule.weights.containsKey(potState.rule.LHS)) {
						float oldWeight = state.rule.weights.get(potState.rule.LHS).floatValue();
						state.rule.weights.put(potState.rule.LHS, new Float (oldWeight + 1));
					}
					else {
						state.rule.weights.put(potState.rule.LHS, new Float (1));
					}

				}

			}
		}
		return chart;
	}

	private static void log(String str) {
		System.out.println("[CFG] " + str);
	}



	public GenerationResult generateWeightedRandomFromNT(String NT, String previousNT, 
			boolean withSemantics) {

		GenerationResult gr = new GenerationResult();

		if (!Rules.containsKey(NT)) {
			return gr;
		}

		Vector<Rule> RHSs = Rules.get(NT);

		Random rand = new Random();
		
		Hashtable<Integer,Integer> weightedChoiceList = new Hashtable<Integer,Integer>();
		int count = 0;
		for (int i=0; i < RHSs.size(); i++) {
			weightedChoiceList.put(new Integer(count), new Integer(i));
			count++;
			Rule rule = RHSs.elementAt(i);
			for (int j=0; rule.weights.containsKey(previousNT) && 
			j <= rule.weights.get(previousNT) ; j++) {
				weightedChoiceList.put(new Integer(count), new Integer(i));
				count++;
			}
		}

		boolean ruleIsChosen = false;
		Rule rule = null;

		// We have to ensure that the same nominal variable never occur twice in the
		// same semantics, so we may have to considure multiple choicses
		
		while (!ruleIsChosen) {
			int weightedChoice = rand.nextInt(weightedChoiceList.keySet().size());
			int choice = weightedChoiceList.get(new Integer(weightedChoice)).intValue();
			rule = RHSs.elementAt(choice);

			if (withSemantics) {
			String[] split = rule.semantics.split(":");
			if (split.length > 0) {
				if (!gr.getSemantics().contains(split[0])) {
					ruleIsChosen = true;
				}
				else {
					log("Found inappropriate rule!");
				}
			}
			else {
				ruleIsChosen = true;
			}
			}
			else {
				ruleIsChosen = true;
			}
		}

		if (withSemantics ) {
			gr.setSemantics(rule.getSemantics());
		}
		String semantics = gr.getSemantics();


		int incr = 1;
		for (Enumeration<String> e = rule.getRHS().elements() ; e.hasMoreElements() ;) {
			String constituent = e.nextElement();

			char firstChar = constituent.charAt(0);
			boolean isNT = (((Character.isUpperCase(firstChar) & 
					(!isClassBased || !constituent.contains("_LC"))) || firstChar == '.') 
			);

			if (isNT) {
				GenerationResult oldGr = generateWeightedRandomFromNT(constituent, NT, withSemantics);
				gr.updateUtterance(oldGr.getUtterance());

				String index = "@"+incr;
				if (withSemantics && semantics.contains(index)) {
					gr.updateSemantics(index, oldGr.getSemantics());
				}
			}
			else {
				gr.updateUtterance(" " + constituent); 
			}
			incr++;
		}
		return gr;
	}

	/** 
	 * Generate a random string using the FST
	 * @return
	 */
	public String generateWeightedRandom() {	
		GenerationResult result = generateWeightedRandomFromNT(StartSymbol, "earleyStart", false);

		//	log(result + ": " + recognize(result));
		return result.getUtterance();
	}

	/** 
	 * Generate a random string using the FST
	 * @return
	 */
	public String generateWeightedRandomWithSemantics() {	
		GenerationResult result = generateWeightedRandomFromNT(StartSymbol, "earleyStart", true);

		String semantics = result.getSemantics();

		String land = new Character((char)94).toString();
		semantics = semantics.replaceFirst(" \\^ ", "(");
		semantics = "@" + semantics + ")";

		result.setSemantics(semantics);

		boolean isOK = true ;

		if (isOK) {
			return result.getUtterance() + ": " + semantics;
		}
		else {
			return "";
		}
	}



	public void setWeights(Vector<String> utterances) {
		weightCountActivated = true;
		for (Enumeration<String> e = utterances.elements() ; e.hasMoreElements() ;) {
			String utt = e.nextElement();
			recognize(utt);
		}
		weightCountActivated = false;

		/**
		for (Iterator<String> e = Rules.keySet().iterator() ; e.hasNext() ;) {
			float totalCount = 0.0f;
			String LHS = e.next();
			Vector<Rule> RHSs = Rules.get(LHS);
			for (Enumeration<Rule> f = RHSs.elements() ; f.hasMoreElements() ; ) {
				Rule rule = f.nextElement();
				totalCount += rule.weight;
			}

			for (Enumeration<Rule> f = RHSs.elements() ; f.hasMoreElements() ; ) {
				Rule rule = f.nextElement();
				if (totalCount > 0) {
					rule.weight = (float)rule.weight / totalCount;
						}
			}
		}*/

	}

	public void addCFG(CFG cfg) {
		Hashtable<String,Vector<Rule>> newRules = cfg.getRules();
		for (Enumeration<String> e = newRules.keys() ; e.hasMoreElements();) {
			String key = e.nextElement();
			Vector<Rule> rule = newRules.get(key);
			Rules.put(key, rule);
		}

	}
}

final class EarleyState {

	static int count = 0;
	String stateId;
	Rule rule;
	int posInRule;
	int chartStart;
	int chartEnd;
	Vector<String> pointers;

	public EarleyState (Rule rule, int posInRule, int chartStart, int chartEnd) {
		this.rule = rule;
		this.posInRule = posInRule;
		this.chartStart = chartStart;
		this.chartEnd = chartEnd;
		stateId = "S" + count;
		count++;
		pointers = new Vector<String>();
	}
	
	static String alreadyExpanded = "";

	public String getSemantics(Chart chart) {
		String semantics = "";
		if (rule.getSemantics()!= null) {
			semantics = rule.getSemantics();
		}
		else {
			semantics = "@1";
		}
		for (Enumeration<String> e = pointers.elements(); e.hasMoreElements();) {
			String pointer = e.nextElement();
			EarleyState state = chart.getState(pointer);
			alreadyExpanded += semantics;
			String subsemantics = state.getSemantics(chart);
			String index = "@"+(rule.RHS.indexOf(state.rule.LHS)+1);
			String[] split = subsemantics.split(":");
			if (split.length >1 && alreadyExpanded.contains(split[0])) {
		//		System.out.println("REPLACING.... " + split[0]);
				subsemantics = subsemantics.replace(split[0], split[0]+"b");
			}
 			if (semantics.contains(index))
				semantics = semantics.replace(index, subsemantics);
			else {
				index = "@"+(rule.RHS.lastIndexOf(state.rule.LHS)+1);
				semantics = semantics.replace(index, subsemantics);
			}
		}
		return semantics;
	}
	
	public boolean isComplete() {
		return posInRule == rule.getRHS().size();
	}

	public boolean isnextConstituentNonTerminal() {
		if (rule.getRHS().size() > posInRule) {
			String el = rule.getRHS().elementAt(posInRule);
			char firstChar = el.charAt(0);
			if (rule.LHS.equals("PERSON_LC")) {
				return false;
			}
			return (((Character.isUpperCase(firstChar) & 
					(!CFG.isClassBased || !el.contains("_LC")) ) || firstChar == '.') );
		}
		return false;
	}

	public String getNextConstituent() {
		if (rule.getRHS().size() > posInRule) {
			String el = rule.getRHS().elementAt(posInRule);
			return el;
		}
		return null;
	}

	public String toString() {
		String result = rule.LHS + " ==> ";
		int count = 0 ;
		for (Enumeration<String> e = rule.RHS.elements() ; e.hasMoreElements() ; ) {
			if (posInRule == count) {
				result += " * ";
			}
			result += e.nextElement() + " ";

			count ++;
		}
		if (posInRule == count) {
			result += " * ";
		}
		result += " [" + chartStart + ","+ chartEnd + "]"; 
		return result;
	}

	public void addPointer(String str) {
		pointers.add(str);
	}

	public void addPointers(Vector<String> strs) {
		pointers.addAll(strs);
	}
}

final class Chart {

	Hashtable<Integer,Vector<EarleyState>> chart;

	public Chart() { 
		chart = new Hashtable<Integer,Vector<EarleyState>>();
	}


	public void enqueueState(EarleyState state, int pos) {
		if (chart.containsKey(new Integer(pos))) {
			Vector<EarleyState> states = chart.get(new Integer(pos));
			for (Enumeration<EarleyState> e = states.elements() ; e.hasMoreElements() ;) {
				EarleyState eState = e.nextElement();
				if (eState.rule.equals(state.rule) && eState.posInRule == state.posInRule &&
						eState.chartStart == state.chartStart && eState.chartEnd == state.chartEnd) {
					return ;
				}
			}
			states.add(state);
			chart.put(new Integer(pos), states);
		}
		else {
			Vector<EarleyState> states = new Vector<EarleyState>();
			states.add(state);
			chart.put(new Integer(pos), states);			
		}
	}

	public Vector<EarleyState> getChartContent(int pos) {
		if (chart.containsKey(new Integer(pos))) {
			Vector<EarleyState> states = chart.get(new Integer(pos));
			return states;
		}
		else {
			//	System.out.println("[CHART] no content at pos " + pos);
			return new Vector<EarleyState>();
		}
	}

	public EarleyState getSimilarState (int pos, EarleyState state) {
		if (chart.containsKey(new Integer(pos))) {
			Vector<EarleyState> states = chart.get(new Integer(pos));
			for (Enumeration<EarleyState> e = states.elements() ; e.hasMoreElements() ;) {
				EarleyState aState = e.nextElement();
				if (aState.rule.equals(state.rule) && aState.posInRule == state.posInRule 
						&& aState.chartStart == state.chartStart && aState.chartEnd == state.chartEnd) {
					return aState;
				}
			}
		}
		return null;
	}

	public EarleyState getState(String stateId) {
		for (Enumeration<Integer> e = chart.keys(); e.hasMoreElements() ;) {
			for (Enumeration<EarleyState> f = chart.get(e.nextElement()).elements() ; f.hasMoreElements() ;) {
				EarleyState state = f.nextElement();
				if (state.stateId.equals(stateId)) {
					return state;
				}
			}
		}
		return null;
	}

	public Vector<EarleyState> getStates(Vector<String> stateIds) {
		Vector<EarleyState> states = new Vector<EarleyState>();
		for (Enumeration<String> e = stateIds.elements() ; e.hasMoreElements() ;) {
			String stateId = e.nextElement();
			EarleyState state = getState(stateId);
			if (state!= null) {
				states.add(state);
			}
		}
		return states;
	}

}

final class GenerationResult {

	String currentUtterance = "";

	String currentSemantics = "";

	public void setUtterance(String utt) {
		currentUtterance = utt;
	}

	public void setSemantics(String semantics) {
		currentSemantics = semantics;
	}

	public String getUtterance() {
		return currentUtterance;
	}

	public String updateSemantics(String index, String semantics) {
		if (currentSemantics.contains(index)) {
			currentSemantics = currentSemantics.replace(index, semantics);
		}
		else {
			log("Warning: index not contained in current semantics");
			log("Utterance: " + currentUtterance);
			log("Semantics: " + currentSemantics);
			log("Index: " + index);
		}
		return currentSemantics;
	}

	public void updateUtterance(String addition) {
		currentUtterance = currentUtterance + addition;
	}

	public String getSemantics() {
		return currentSemantics;
	}

	public void log(String str) {
		System.out.println("[GENERATION RESULT] " + str);
	}


}
