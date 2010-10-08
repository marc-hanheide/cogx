//=================================================================
//Copyright (C) 2010 Pierre Lison (plison@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.util;

//=================================================================
// IMPORTS

// Java
import java.util.*;

// Dialogue API
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;
import de.dfki.lt.tr.dialogue.util.DialogueInvalidOperationException;


/**
 * The class <tt>CFG</tt> provides a basic definition of a context-free grammar of
 * the form LHS :- RHS1...RHSn, whereby there may be more than one rule with a given LHS.
 * The class provides functionality for Early-based recognition and parsing with a CFG in
 * this format, and it can be used to generate utterances with semantics.
 * <p>
 * The class includes several inline class definitions for Early state, chart, and generation result.
 *
 * @author 	Pierre Lison (plison@dfki.de)
 * @author  Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100608
 */

public class CFG {
	/** the start symbol */
	String StartSymbol;
	/** the context-free rules, indexed by start symbol */
	Hashtable<String,Vector<Rule>> Rules ;
	/** whether to modify the rules weights during recognition */
	boolean weightCountActivated = false;
	/** flag whether the grammar is class-based or not */
	public static boolean isClassBased = false;

	/**
	 * Constructor
	 */
	public CFG () {
		Rules = new Hashtable<String,Vector<Rule>>();
	} // end constructor


	/**
	 * Set the CFG start symbol
	 * @param symbol	The symbol to be used
	 */
	public void setStartSymbol(String symbol) {
		StartSymbol = symbol;
	} // end

	/**
	 * Add a new context-free rule, composed of a left-hand side and a right-hand side. The
	 * method allows for the grammar to have rules with identical LHSs.
	 *
	 * @param LHS	The left-hand side of the rule, as a single symbol
	 * @param RHS	The right-hand side of the rule, as a list (vector) of string symbols
	 * @throws DialogueMissingValueException Thrown if LHS or RHS are empty or null
	 */
	public void addRule(String LHS, Vector<String> RHS)
	throws DialogueMissingValueException
	{
		if (LHS == null || LHS.equals(""))
		{
			throw new DialogueMissingValueException("Cannot construct CFG rule: LHS empty or null");
		}
		if (RHS == null ||RHS.size() == 0)
		{
			throw new DialogueMissingValueException("Cannot construct CFG rule: RHS empty or null");
		}
		if (Rules.containsKey(LHS)) {
			Vector<Rule> curRHSs = Rules.get(LHS);
			curRHSs.add(new Rule(LHS, RHS));
			Rules.put(LHS, curRHSs);
		}
		else {
			Vector<Rule> newRHSs = new Vector<Rule>();
			newRHSs.add(new Rule(LHS, RHS));
			Rules.put(LHS, newRHSs);
		} // end if..else check whether known LHS or not
 	} // end addRule

	/**
	 * Add a new context-free rule, given a Rule object
	 * @param rule	The rule to be added
	 * @throws DialogueMissingValueException Thrown if the object is null
	 */
	public void addRule(Rule rule)
	throws DialogueMissingValueException
	{
		if (rule == null)
		{
			throw new DialogueMissingValueException("Cannot construct CFG rule: Rule object null");
		}
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
	} // end addRule

	/**
	 * Adds a collection of rules, given as a hashtable <LHS, <RHS>>
	 * @param rules
	 * @throws DialogueMissingValueException Thrown if rules table is empty or null
	 */

	public void addRules(Hashtable<String,Vector<Rule>> rules)
	throws DialogueMissingValueException
	{
		if (rules == null || rules.size() == 0)
		{
			throw new DialogueMissingValueException("Cannot construct rules: Table object null or empty");
		}
		for (Enumeration<String> e = rules.keys() ; e.hasMoreElements() ; ) {
			String LHS = e.nextElement();
			for (Enumeration<Rule> f = rules.get(LHS).elements() ; f.hasMoreElements() ;) {
				addRule(f.nextElement());
			} // end for over elements - RHSs for LHS
		} // end for over rule keys - LHS
	} // end addRules


	/**
	 * Adds rules on the basis of a LHS key, and possible RHSs (as strings) for this LHS
	 *
	 * @param LHS	The LHS key of the rule
	 * @param RHSs	The set of possible RHSs for the LHS key
	 * @throws DialogueMissingValueException Thrown if the LHS or the RHSs are empty
	 */

	public void addRules(String LHS,  Vector<Vector<String>> RHSs)
	throws DialogueMissingValueException
	{
		if (LHS == null || LHS.equals(""))
		{
			throw new DialogueMissingValueException("Cannot construct rules: LHS is empty or null");
		}
		if (RHSs == null || RHSs.size() == 0)
		{
			throw new DialogueMissingValueException("Cannot construct rules: RHS is empty or null");
		}
		for (Enumeration<Vector<String>> e = RHSs.elements() ; e.hasMoreElements() ; ) {
			addRule(LHS, e.nextElement());
		}
	} // end addRules

	/**
	 * Adds rules on the basis of a LHS key, and a vector of Rule objects
	 * @param LHS		The LHS key
	 * @param rules		A vector of Rule objects
	 * @throws DialogueMissingValueException Thrown if LHS or rules are empty,
	 * 			(or if any of the rules themselves are null)
	 */

	public void addRules2(String LHS, Vector<Rule> rules)
	throws DialogueMissingValueException
	{
		if (LHS == null || LHS.equals(""))
		{
			throw new DialogueMissingValueException("Cannot construct rules: LHS is empty or null");
		}
		if (rules == null || rules.size() == 0)
		{
			throw new DialogueMissingValueException("Cannot construct rules: RHS rules are empty or null");
		}
		for (Enumeration<Rule> e = rules.elements() ; e.hasMoreElements() ; ) {
			Rule RHS = e.nextElement();
			RHS.setLHS(LHS);
			addRule(RHS);
		}
	} // end addRules2

	/**
	 * Returns the start symbol of the grammar
	 * @return String The start symbol
	 */

	public String getStartSymbol()  {
		return StartSymbol;
	} // end getStartSymbol

	/**
	 * Returns the rules of the grammar, as a table of LHS keys, and lists (Vector) of Rule objects
	 * @return Hashtable 	The rules table
	 */

	public Hashtable<String,Vector<Rule>> getRules() {
		return Rules;
	} // end getRules

	/**
	 * Returns a string representation of the grammar
	 * @return String A string representation of the grammar
	 */
	@Override
  public String toString() {
		String result = "CFG grammmar:\n";
		for (Enumeration<String> e = Rules.keys() ; e.hasMoreElements() ; ) {
			String key = e.nextElement();
			for (Enumeration<Rule> f= Rules.get(key).elements() ; f.hasMoreElements() ;) {
				Rule rule = f.nextElement();
				result += rule.toString() + "\n";
			} // end for over rules (RHS)
		} // end for over keys (LHS)
		return result;
	} // end toString

	/**
	 * Returns a boolean indicating whether the given string is recognized by the grammar or not.
	 * If the string (and the grammar) includes punctuation, each punctuation must be represented
	 * as an individual word (i.e. surrounded by white space, not connected to a word).
	 *
	 * @param  str		The string to be parsed
	 * @return boolean 	Whether or not the string is recognized by the grammar
	 * @throws DialogueInvalidOperationException	Thrown if there is no start symbol for the grammar
	 * @throws DialogueMissingValueException		Thrown if the provided string is null
	 */

	public boolean recognize(String str)
	throws DialogueInvalidOperationException, DialogueMissingValueException
	{
		if (str == null)
		{
			throw new DialogueMissingValueException("Cannot recognize string: Provided string is null");
		}
		if (StartSymbol == null) {
			throw new DialogueInvalidOperationException("Cannot recognize string: "+
					"No start symbol provided for grammar");
		}
		// Set up the Early state
		Vector<String> earleyRHS = new Vector<String>();
		earleyRHS.add(StartSymbol);
		Rule earleyStartrule = new Rule("earleyStart", earleyRHS);
		EarleyState earleyStartState = new EarleyState (earleyStartrule, 0, 0, 0);
		// Set up the chart, initialize with the start state
		Chart chart = new Chart();
		chart.enqueueState(earleyStartState, 0);
		// Split the string into individual words
		String[] words = str.split(" ");
		// Initialize the chart with the words, then construct the chart using predict/scan/complete
		chart = retrieveChart(words, chart);
		// Formulate successful endstates
		EarleyState successfullEndState = new EarleyState(earleyStartrule, 1, 0, words.length);
		EarleyState successfulParse = chart.getSimilarState(words.length, successfullEndState);
		//
		if (successfulParse != null) {
			Vector<EarleyState> pointers = chart.getStates(successfulParse.pointers);
			while (pointers.size() != 0) {
				EarleyState state = pointers.remove(pointers.size() -1);
				pointers.addAll(chart.getStates(state.pointers));
			} // end while
		} // end while
		return successfulParse != null;
	} // end recognize

	/**
	 * Returns the semantics for a string, given a successful parse using the CFG. The
	 * semantics is empty if no parse is found (or if that's all the grammar assigns).
	 *
	 * @param 	str		The string to be parsed, and given a semantics to
	 * @return	String	An "HLDS"-type semantic representation for the string
	 * @throws DialogueMissingValueException		Thrown if the provided string is null
	 * @throws DialogueInvalidOperationException	Thrown if the grammar has no start symbol
	 */

	public String getSemantics(String str)
	throws DialogueMissingValueException, DialogueInvalidOperationException
	{
		if (str == null)
		{
			throw new DialogueMissingValueException("Cannot get semantics for string: "+
					"String is null");
		}
		if (StartSymbol == null) {
			throw new DialogueInvalidOperationException("Cannot get semantics for string: "+
					"No start symbol defined for grammar");
		}
		// Initialize
		EarleyState.alreadyExpanded = "";
		Vector<String> earleyRHS = new Vector<String>();
		earleyRHS.add(StartSymbol);
		Rule earleyStartrule = new Rule("earleyStart", earleyRHS);
		EarleyState earleyStartState = new EarleyState (earleyStartrule, 0, 0, 0);
		// Initialize the chart with the state
		Chart chart = new Chart();
		chart.enqueueState(earleyStartState, 0);
		// Initialize the list of words from the string
		String[] words = str.split(" ");
		// Initialize the chart and parse
		chart = retrieveChart(words, chart);
		EarleyState successfullEndState = new EarleyState(earleyStartrule, 1, 0, words.length);
		EarleyState successfulParse = chart.getSimilarState(words.length, successfullEndState);
		if (successfulParse != null) {
			Vector<EarleyState> pointers = chart.getStates(successfulParse.pointers);
			while (pointers.size() != 0) {
				EarleyState state = pointers.remove(pointers.size() -1);
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
	} // end getSemantics

	/**
	 * Constructs a complete chart for a given string, using Early-style predict, scan and
	 * complete steps.
	 * @param words		The list of words to be parsed
	 * @param chart		An initialized chart
	 * @return Chart	The completed chart
	 * @throws DialogueMissingValueException	Thrown if either provided words or chart are empty
	 */

	public Chart retrieveChart(String[] words, Chart chart)
	throws DialogueMissingValueException
	{
		if (words == null)
		{
			throw new DialogueMissingValueException("Cannot retrieve chart: Words are null");
		}
		if (chart == null)
		{
			throw new DialogueMissingValueException("Cannot retrieve chart: Provided chart is null");
		}
		for (int i = 0 ; i <= words.length ; i++) {
			for (Enumeration<EarleyState> e = chart.getChartContent(i).elements(); e.hasMoreElements();) {
				EarleyState state = e.nextElement();
				if (!state.isComplete() && state.isnextConstituentNonTerminal()) {
					chart = predictor(state, chart);
				}
				else if (!state.isComplete() && i < words.length && ! state.isnextConstituentNonTerminal()) {
					chart = scanner(state,chart, words[i]);
				}
				else if (state.isComplete()){
					chart = completer(state, chart);
				} // end if..else check whether to predict, scan or complete
			} // end for over states
		} // end for over words
		return chart;
	} // end retrieveChart

	/**
	 * Predicts rules to be applied, based on what RHS rules the next constituent as LHS triggers
	 * The chart is updated with additional Early states, one for each predicted rule.
	 * @param state		The current state
	 * @param chart		The current chart
	 * @return Chart	The chart, possibly updated with prediction steps
	 * @throws DialogueMissingValueException Thrown if either state or chart is missing
	 */

	private Chart predictor(EarleyState state, Chart chart)
	throws DialogueMissingValueException
	{
		if (state == null)
		{
			throw new DialogueMissingValueException("Cannot predict next states: current state is null");
		}
		if (chart == null)
		{
			throw new DialogueMissingValueException("Cannot predict next states: current chart is null");
		}
		// Get the next constituent from the state, and see whether any rules apply
		// If any do apply, use the RHSs as predictors, generating new states
		String nextConstituent = state.getNextConstituent();
		if (Rules.containsKey(nextConstituent)) {
			for (Enumeration<Rule> e = Rules.get(nextConstituent).elements() ; e.hasMoreElements() ; ) {
				EarleyState newState = new EarleyState (e.nextElement(), 0, state.chartEnd, state.chartEnd);
				chart.enqueueState(newState, state.chartEnd);
			} // end for over rules
		} // end if.. check whether next constituent is in LHS
		return chart;
	} // end predict

	/**
	 * Scans over the next constituent as word, if so generates a new state using the rule
	 *
	 * @param state		The current state
	 * @param chart		The current chart
	 * @param word		The word to scan over
	 * @return	Chart	A chart, possibly updated with new states
	 * @throws 	DialogueMissingValueException Thrown if any parameters are null or empty
	 */

	private Chart scanner(EarleyState state, Chart chart, String word)
	throws DialogueMissingValueException
	{
		if (state == null)
		{
			throw new DialogueMissingValueException("Cannot scan in state: state is null");
		}
		if (chart == null)
		{
			throw new DialogueMissingValueException("Cannot scan in state: chart is null");
		}
		if (word == null || word.equals(""))
		{
			throw new DialogueMissingValueException("Cannot scan in state: word is null or empty");
		}
		if(state.getNextConstituent().equals(word)) {
			EarleyState newState = new EarleyState(state.rule, state.posInRule + 1,
					state.chartStart, state.chartEnd + 1);
			newState.addPointers(state.pointers);
			chart.enqueueState(newState, state.chartEnd+1);
		} // if..check whether word is the next constituent
		return chart;
	} // end scanner

	/**
	 * Competes which states are possibly completing "over" the current position
	 * @param state		The current state
	 * @param chart		The current chart
	 * @return	Chart	A chart, possibly updated with new completed states
	 * @throws DialogueMissingValueException
	 */

	private Chart completer(EarleyState state, Chart chart)
	throws DialogueMissingValueException
	{
		if (state == null)
		{
			throw new DialogueMissingValueException("Cannot complete: state is null");
		}
		if (chart == null)
		{
			throw new DialogueMissingValueException("Cannot complete: chart is null");
		}
		// Retrieve the states at position J
		Vector<EarleyState> chartAtPosJ = chart.getChartContent(state.chartStart);
		// Cycle over the states at the position
		for (Enumeration<EarleyState> e = chartAtPosJ.elements() ; e.hasMoreElements() ; ) {
			EarleyState potState = e.nextElement();
			// Check whether the state completes at this point
			if (potState.getNextConstituent() != null &&
					potState.getNextConstituent().equals(state.rule.LHS) &&
					potState.chartEnd == state.chartStart) {
				EarleyState newState = new EarleyState(potState.rule,
						potState.posInRule + 1, potState.chartStart, state.chartEnd);
				newState.addPointers(potState.pointers);
				newState.addPointer(state.stateId);
				chart.enqueueState(newState, state.chartEnd);
				// if using recognition to initialize weights, update the weights for the rule
				if (weightCountActivated) {
					if (state.rule.weights.containsKey(potState.rule.LHS)) {
						float oldWeight = state.rule.weights.get(potState.rule.LHS).floatValue();
						state.rule.weights.put(potState.rule.LHS, new Float (oldWeight + 1));
					}
					else {
						state.rule.weights.put(potState.rule.LHS, new Float (1));
					} // end if..else check whether to update existing rule weight, or start new count
				} // end if..
			} // end if.. check for potentially completing state
		} // end for over states at this position
		return chart;
	} // end completer

	/**
	 * Generates an utterance from a given nonterminal, using a weighted random choice from the rules
	 * triggered by the nonterminal. The weighted choice is implemented by constructing a choice list,
	 * which stores <count, rule number> while cycling over applicable RHSs. Conditioning on a
	 * previous (preceding) nonterminal can increase the counting for a specific, applicable RHS.
	 * A random choice is then finally made over the keys (counts) to select a rule (stored as value).
	 *
	 * @param NT				A non-empty nonterminal from which the start generating
	 * @param previousNT		A non-empty nonterminal on which the condition application of NT rules
	 * @param withSemantics		Flag whether to generate with semantics
	 * @return GenerationResult	The result, including an utterance and a semantics
	 * @throws DialogueMissingValueException 	 Thrown if any parameter is null or empty
	 * @throws DialogueInvalidOperationException Thrown if the NT is not in the grammar
	 */

	public GenerationResult generateWeightedRandomFromNT(String NT, String previousNT,
			boolean withSemantics)
	throws DialogueInvalidOperationException
	{
		if (NT == null || NT.equals(""))
		{
			throw new DialogueMissingValueException("Cannot generate randomly from nonterminal: "+
					"NT is null or empty");
		}
		if (previousNT == null || previousNT.equals(""))
		{
			throw new DialogueMissingValueException("Cannot generate randomly from nonterminal: "+
					"previous NT is null or empty");
		}
		if (!Rules.containsKey(NT)) {
			throw new DialogueInvalidOperationException ("Cannot generate randomly from nonterminal: "+
					"Provided NT not in the grammar");
		}
		// Initialize the result
		GenerationResult gr = new GenerationResult();
		// Get the RHSs triggered by NT as LHS
		Vector<Rule> RHSs = Rules.get(NT);
		// Initialize a new randomizer
		Random rand = new Random();
		// Initialize a weighted choice list
		Hashtable<Integer,Integer> weightedChoiceList = new Hashtable<Integer,Integer>();
		int count = 0;
		// Cycle over the RHSs
		for (int i=0; i < RHSs.size(); i++) {
			// Update the weighted choice list with a count (increasing) and the position in the list
			weightedChoiceList.put(new Integer(count), new Integer(i));
			count++;
			Rule rule = RHSs.elementAt(i);
			// Cycle over the rule weights, assuming previous NT is predecessor
			// and update the counts for the applied rule
			for (int j=0; rule.weights.containsKey(previousNT) &&
			j <= rule.weights.get(previousNT) ; j++) {
				weightedChoiceList.put(new Integer(count), new Integer(i));
				count++;
			} // end for over rule weights
		} // end for over the RHSs
		boolean ruleIsChosen = false;
		Rule rule = null;
		// We have to ensure that the same nominal variable never occur twice in the
		// same semantics, so we may have to consider multiple choices
		while (!ruleIsChosen) {
			// choose an arbitrary number in the range of the weighted choice list size
			int weightedChoice = rand.nextInt(weightedChoiceList.keySet().size());
			// choose that rule (which, the more often it gets applied, the more
			// likely it is to be chosen
			int choice = weightedChoiceList.get(new Integer(weightedChoice)).intValue();
			rule = RHSs.elementAt(choice);
			//
			if (withSemantics) {
				String[] split = rule.semantics.split(":");
				if (split.length > 0) {
					if (!gr.getSemantics().contains(split[0])) {
						ruleIsChosen = true;
					}
					else {
						throw new DialogueInvalidOperationException("Cannot generate randomly from nontermal: "+
						"Found inappropriate rule in grammar when generating with semantics");
					} // end if..else check for appropriate rule
				}
				else {
					ruleIsChosen = true;
				} // end if..else check whether split correct
			}
			else
			{
				ruleIsChosen = true;
			} // end if..else check whether with semantics
		} // end while
		// add the semantics to the generation result
		if (withSemantics ) {
			gr.setSemantics(rule.getSemantics());
		}
		String semantics = gr.getSemantics();
		// next, generate the utterance by starting from the NT, call recursively
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
	} // end generateWeightedRandomFromNT

	/**
	 * Generate a random string using the grammar. The string is generated using the grammar's
	 * starting symbol as initial non-terminal. No semantics is provided
	 *
	 * @return String	The randomly generated string
	 * @throws DialogueInvalidOperationException 	Thrown if the grammar has no start symbol
	 */
	public String generateWeightedRandom()
	throws DialogueInvalidOperationException
	{
		if (StartSymbol == null || StartSymbol.equals(""))
		{
			throw new DialogueInvalidOperationException("Cannot generate weighted random string: "+
					"No start symbol defined for grammar");
		}
		GenerationResult result = generateWeightedRandomFromNT(StartSymbol, "earleyStart", false);
		return result.getUtterance();
	}

	/**
	 * Generate a random string and its semantics using the grammar. The result is provided
	 * as a String "utterance : semantics" with semantics in "HLDS" format.
	 *
	 * @return String	The randomly generated string, with semantics
	 * @throws DialogueInvalidOperationException Thrown if the grammar has no start symbol
	 */
	public String generateWeightedRandomWithSemantics()
	throws DialogueInvalidOperationException
	{
		if (StartSymbol == null || StartSymbol.equals(""))
		{
			throw new DialogueInvalidOperationException("Cannot generate weighted random string: "+
					"No start symbol defined for grammar");
		}
		GenerationResult result = generateWeightedRandomFromNT(StartSymbol, "earleyStart", true);
		String semantics = result.getSemantics();
		String land = new Character((char)94).toString();
		semantics = semantics.replaceFirst(" \\^ ", "(");
		semantics = "@" + semantics + ")";
		result.setSemantics(semantics);
		// return the result
		return result.getUtterance() + ": " + semantics;
	} // end generateWeightedRandomWithSemantics

	/**
	 * Sets the weights for rules in the grammar, given a set of utterances used as seed. Weights
	 * are updated by checking their application while recognizing utterances in the given set.
	 * This update is done in the completer step of the Early-based parsing/recognition.
	 *
	 * @param utterances	The utterances to be used as seed
	 * @throws DialogueMissingValueException		Thrown if the vector is null or empty
	 * @throws DialogueInvalidOperationException	Passed on "recognize"
	 * @see	CFG#recognize(String)
	 * @see CFG#completer(EarleyState, Chart)
	 */

	public void setWeights(Vector<String> utterances)
	throws DialogueMissingValueException, DialogueInvalidOperationException
	{
		if (utterances == null || utterances.size() == 0)
		{
			throw new DialogueMissingValueException("Cannot set weights: Provided utterances null or empty");
		}
		weightCountActivated = true;
		for (Enumeration<String> e = utterances.elements() ; e.hasMoreElements() ;) {
			String utt = e.nextElement();
			recognize(utt);
		}
		weightCountActivated = false;
	} // end setWeights

	/**
	 * Adds a complete CFG to the local CFG
	 * @param cfg	The CFG to be added
	 * @throws DialogueMissingValueException Thrown if the provided cfg is null
	 */

	public void addCFG(CFG cfg)
	throws DialogueMissingValueException
	{
		if (cfg == null)
		{
			throw new DialogueMissingValueException("Cannot add CFG to CFG: Provided CFG is null");
		}
		Hashtable<String,Vector<Rule>> newRules = cfg.getRules();
		for (Enumeration<String> e = newRules.keys() ; e.hasMoreElements();) {
			String key = e.nextElement();
			Vector<Rule> rule = newRules.get(key);
			Rules.put(key, rule);
		} // end for over rules
	} // end addCFG

} // end class

/**
 *
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	100608
 */


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

	@Override
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
			// log("Warning: index not contained in current semantics");
			// log("Utterance: " + currentUtterance);
			// log("Semantics: " + currentSemantics);
			// log("Index: " + index);
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
