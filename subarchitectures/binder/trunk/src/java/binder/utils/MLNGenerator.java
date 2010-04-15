package binder.utils;

import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
 
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionValues;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.FormulaValues;
import beliefmodels.autogen.distribs.NormalValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.featurecontent.StringValue;
import beliefmodels.autogen.featurecontent.UnknownValue;
import binder.ml.MLException;
import binder.ml.MLFormula;

public class MLNGenerator {

	public static boolean LOGGING = true;
	
	public static boolean DEBUG = true;
	
	private static final String EXISTING_UNIONS = "existingunion";

	public static final String STANDARD_INPUT_ID = "P";
		
	private static final Object NEWLINE = "\n";
	
	MLNPreferences preferences;
	
	private Map<String, Set<String>> names_for_type;
	private Map<String, Set<String>> predicates_for_belief;
	
	private Belief newInput;
	
	public MLNGenerator() {
		names_for_type = new TreeMap<String, Set<String>>();
		predicates_for_belief = new TreeMap<String, Set<String>>();
		preferences = new MLNPreferences();
	}
	
	public MLNGenerator(MLNPreferences preferences) {
		names_for_type = new TreeMap<String, Set<String>>();
		predicates_for_belief = new TreeMap<String, Set<String>>();
		this.preferences = preferences;
	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	// Build the Markov network file
	/////////////////////////////////////////////////////////////////////////////////////
	 
	// we build the corresponding Markov logic network file step by step
	public void writeMLNFile(PerceptBelief b, Collection<PerceptUnionBelief> existingUnions, 
			HashMap<String,String> unionsMapping, String newSingleUnionId, String MLNFileToWrite) throws MLException {
		 
		newInput = b;
		
		StringBuilder mln_file = new StringBuilder();
		
		// we read all the predicates that are specified in the correlation
		// section including all names and add them to names_for_types 
		parseCorrelationPredicates();
		
		// before we can start with the serialization we need to know all the names
		// and types that appear in the beliefs
		Set<String> predicates_for_current_belief = new TreeSet<String>();
		predicates_for_belief.put(STANDARD_INPUT_ID, predicates_for_current_belief);
		extractTypesAndNames(b.content, predicates_for_current_belief);
		
		// for all the remaining ones
		for(Belief belief : existingUnions) {
			predicates_for_current_belief = new TreeSet<String>();
			predicates_for_belief.put(getMarkovLogicConstantFromID(belief.id), predicates_for_current_belief);
			extractTypesAndNames(belief.content, predicates_for_current_belief);
		}
		
		// now lets do the serialization
		mln_file.append(constructConstantsSection(existingUnions, unionsMapping, newSingleUnionId));
		mln_file.append(getPredicatesSection());
		mln_file.append(constructFormulaeForExistingUnions(existingUnions));
		mln_file.append(constructFormulaeForPercept(b));
		mln_file.append(getFeatValueConstraintsSection());
		mln_file.append(getCorrelationSection());
		mln_file.append(extractFinalOutcomeSection(existingUnions, unionsMapping, newSingleUnionId));
		
		FileUtils.writeFile(MLNFileToWrite, mln_file.toString());
	}
	
	private void parseCorrelationPredicates() {
		String content = FileUtils.readfile(this.preferences.getFile_predicates());
		String[] lines = content.split("\n");
		for(String line : lines) {
			String[] tokens = line.split(":");
			
			if(tokens.length == 2) {
				String predicate = setFirstLetterToUppercase(tokens[0].trim());
				
				Set<String> names = new TreeSet<String>();
				for(String name : tokens[1].split("\\s")) {
					if(name.trim().length() >= 1) {
						names.add(setFirstLetterToUppercase(name.trim()));
						log("adding: " + predicate + " with name " + name);
					}
				}
				
				if(names_for_type.containsKey(predicate)) {
					names_for_type.get(predicate).addAll(names);
				}
				else {
					names_for_type.put(predicate, names);
				}
			}
			else {
				log("skipping ling " + line + "while parsing correlation tokens");
			}
		}
	}

	// 1) constants section 
	private StringBuilder constructConstantsSection (Collection<PerceptUnionBelief> existingUnions, 
			HashMap<String,String> unionsMapping, String newSingleUnionId) {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// CONSTANTS\n");
		result.append("///////////////////////////////\n");
		
		// 1.1) beliefs
		result.append(NEWLINE);
		result.append("// beliefs\n");
		result.append(serializeBeliefs(existingUnions));
		
		// 1.2) possible new outcomes
		result.append(NEWLINE);
		result.append("// possible outcomes of the grouping process\n");
		result.append(serializeOutcomes(unionsMapping, newSingleUnionId));
		
		// 1.3) feature values
		result.append(NEWLINE);
		result.append("// feature values\n");
		result.append(serializeFeatureValues());
		
		result.append(NEWLINE);
		return result;
	}
	
	// 2) predicate section
	private StringBuilder getPredicatesSection() {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// PREDICATES\n");
		result.append("///////////////////////////////\n");
		
		result.append(NEWLINE);
		for(String predicate_type : names_for_type.keySet()) {
			result.append(predicate_type + "(belief," + setFirstLetterToLowercase(predicate_type) + "val)\n");
		}
		
		result.append(NEWLINE);
		result.append("Outcome(outcome)\n");
		result.append("UnifyWith(existingunion)\n");
		result.append("Existence(belief)\n");
		result.append(NEWLINE);
		return  result;
	}
	
	// 3) existing union section
	private StringBuilder constructFormulaeForExistingUnions (Collection<PerceptUnionBelief> existingUnions) throws MLException {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// EXISTING UNIONS\n");
		result.append("///////////////////////////////\n");
		result.append(NEWLINE);
		
		for(PerceptUnionBelief belief : existingUnions) {
			result.append(serializeBelief(belief));
			result.append(NEWLINE);
		}
		
		return result;
	}
	
	// 4) new percept section
	private StringBuilder constructFormulaeForPercept (PerceptBelief b) throws MLException {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// NEW PERCEPT\n");
		result.append("///////////////////////////////\n");
		result.append(NEWLINE);
		result.append(serializeBelief(b));
		result.append(NEWLINE);
		return result;
	}
	
	// 5) feature value constraints section
	private StringBuilder getFeatValueConstraintsSection () {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// FEATURE VALUE CONSTRAINTS\n");
		result.append("///////////////////////////////\n");
		result.append(NEWLINE);
		
		// a) the serialization of the existence dependency constraints for each predicate/type
		result.append("// rules for existence dependency\n");
		for(String predicate_type : names_for_type.keySet()) {
			result.append("!Existence(x) => y=None v !" + predicate_type + "(x,y).\n");
			result.append("!Existence(x) => " + predicate_type + "(x,None).\n");
		}
		result.append(NEWLINE);
		
		// b) the serialization of the exclusivity and unicinity constraints
		result.append("// mutual exclusivity and unicity constraints\n");
		for(String predicate_type : names_for_type.keySet()) {
			result.append(predicate_type+"(x,y) ^ " + predicate_type + "(x,z) => y=z.\n");
			result.append("EXIST y " + predicate_type + "(x,y).\n");
		}
		result.append(NEWLINE);
		
		return result;
	}
	
	// 6) correlation section
	private String getCorrelationSection () {
		return FileUtils.readfile(this.preferences.getFile_correlations());
	}
	
	// 7) final outcome section
	private StringBuilder extractFinalOutcomeSection(Collection<PerceptUnionBelief> existingUnions, HashMap<String,String> unionsMapping, String singleUnionId) {
		// FIXME: add parameter to control the probability of the outcome
		// FIXME: add parameter to control greediness of the binding process
		// FIXME: @Pierre Is the mapping P -> BELIEFID already in the unionsMapping?
		// because I require it to be there, otherwise we have no clue about the name
		// of the outcome that groups nothing
		
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// FINAL OUTCOME\n");
		result.append("///////////////////////////////\n");
		result.append(NEWLINE);
		
		// a) specification of the different outcomes
		Float greediness = this.preferences.getGreediness();
		result.append(greediness.toString() + " Existence(P) => Outcome(" + getMarkovLogicConstantFromID(singleUnionId) + ")\n");
		
		// then comes the rest of the possible results
		Set<String> new_unions = new TreeSet<String>(unionsMapping.keySet());
		for(String new_union : new_unions) {
			result.append("UnifyWith(" + getMarkovLogicConstantFromID(unionsMapping.get(new_union)) + 
					") <=> Outcome(" + getMarkovLogicConstantFromID(new_union) + ").\n");
		}
		
		// b) mutual exclusivity and unicity
		Float outcome = this.preferences.getOutcome();
		result.append(outcome.toString() + " Outcome(x)\n");
		result.append("Outcome(x) ^ Outcome(y) => x=y.\n");
		result.append("Exist x Outcome(x).\n");
		result.append(NEWLINE);
		
		return result;
	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	// Build the markov network file (helper methods)
	/////////////////////////////////////////////////////////////////////////////////////
	
	private StringBuilder serializeBeliefs(Collection<PerceptUnionBelief> existingUnions) {
		StringBuilder result = new StringBuilder();
		Iterator<PerceptUnionBelief> iter = existingUnions.iterator();
		
		// 1) the existing unions
		String existing_unions = getMarkovLogicConstantFromID(iter.next().id);
		while(iter.hasNext()) {
			existing_unions += "," + getMarkovLogicConstantFromID(iter.next().id);
		}
		result.append(EXISTING_UNIONS + " = {" + existing_unions + "}\n");
		
		// 2) the new current belief
		result.append("percept = {P}\n");
		
		// 3) all possible beliefs
		result.append("belief = {" + existing_unions + ",P}\n");
		
		return result;
	}
	
	private StringBuilder serializeOutcomes(HashMap<String, String> unionsMapping, String newSingleUnionId) {
		StringBuilder result = new StringBuilder();
		result.append("outcome = {");
		 
		Iterator<String> iter = unionsMapping.keySet().iterator();
		result.append(getMarkovLogicConstantFromID(iter.next()));
		while(iter.hasNext()) {
			result.append("," + getMarkovLogicConstantFromID(iter.next()));
		}
		result.append("," + getMarkovLogicConstantFromID(newSingleUnionId));
		result.append("}\n");
		return result;
	}
	
	private StringBuilder serializeFeatureValues() {
		StringBuilder result = new StringBuilder();
		
		for(String type : names_for_type.keySet()) {
			result.append(type.toLowerCase() + "val = {");
			
			Iterator<String> iter = names_for_type.get(type).iterator();
			if(iter.hasNext()) {
				result.append(iter.next());
			}
			while(iter.hasNext()) {
				result.append(",");
				result.append(iter.next());
			}
			
			result.append(",Unknown,None}\n");
		}
		return result;
	}
	
	private void extractTypesAndNames(ProbDistribution distribution, Set<String> predicatesForCurrentBelief) throws MLException {
		// TODO: extend with the remaining distributions
		
		if(distribution instanceof BasicProbDistribution) {
			extractTypesAndNamesBasicProbDistribution((BasicProbDistribution) distribution, predicatesForCurrentBelief);
		}
		
		else if (distribution instanceof DistributionWithExistDep) {
			extractTypesAndNames(((DistributionWithExistDep)distribution).Pc, predicatesForCurrentBelief);	
		}
		
		else if (distribution instanceof CondIndependentDistribs) {
			if (((CondIndependentDistribs)distribution).distribs == null) {
				throw new MLException("Error, distribution is null");
			}
			
			for (String subdistribKey : ((CondIndependentDistribs)distribution).distribs.keySet()) {
				extractTypesAndNames(((CondIndependentDistribs)distribution).distribs.get(subdistribKey), predicatesForCurrentBelief);
			}
		}
		
		else {
			log("Distribution unknown: " + distribution.getClass().getCanonicalName());
			throw new MLException ("Error, distribution is null");
		}
	}
	
	
	private void extractTypesAndNamesBasicProbDistribution(BasicProbDistribution distrib, Set<String> predicatesForCurrentBelief) {
		String keyWithUppercase = setFirstLetterToUppercase(distrib.key);
		if(!names_for_type.containsKey(keyWithUppercase)) {
			names_for_type.put(keyWithUppercase, new TreeSet<String>());
		}
		for (String value: getListDistributionValues(distrib.values)) {
			String valWithUppercase = setFirstLetterToUppercase(value);
			names_for_type.get(keyWithUppercase).add(valWithUppercase);
			predicatesForCurrentBelief.add(valWithUppercase);
		}
	}
	

	private List<String> getListDistributionValues(DistributionValues values) {
		if(values instanceof FeatureValues) {
			return namesAndTypesFeatureValues((FeatureValues) values);
		}
		if(values instanceof NormalValues) {
			return namesAndTypesNormalValues((NormalValues) values);
		}
		if(values instanceof FormulaValues) {
			return namesAndTypesFormulaValues((FormulaValues) values);
		}
		else {
			log("DistributionValues unknown");
		}
		return null;
	}

	private List<String> namesAndTypesFormulaValues(FormulaValues values) {
		// TODO Auto-generated method stub
		return null;
	}

	private List<String> namesAndTypesNormalValues(NormalValues values) {
		// TODO Auto-generated method stub
		return null;
	}

	private List<String> namesAndTypesFeatureValues(FeatureValues values) {
		List<String> result = new LinkedList<String>();
		
		for(FeatureValueProbPair feature_pair : values.values) {
			FeatureValue val = feature_pair.val;
			if(val instanceof StringValue) {
				result.add(((StringValue)val).val);
				continue;
			}
			if(val instanceof IntegerValue) {
				result.add(new Integer(((IntegerValue)val).val).toString());
				continue;
			}
			if(val instanceof BooleanValue) {
				result.add(new Boolean(((BooleanValue)val).val).toString());
				continue;
			}
			if(val instanceof UnknownValue) {
				// we should not add anything here
				// because the unknown value is added implicitly during the construction
				// for each type that appearers in the beliefs
				continue;
			}
			if(val instanceof PointerValue) {
				// TODO: is this correct to translate the belief id to
				// our internal representation?
				result.add(getMarkovLogicConstantFromID(((PointerValue)val).beliefId.id));
				continue;
			}
		}
		return result;
	}

	private StringBuilder serializeBelief(Belief belief) throws MLException {
		List<MLFormula> formulae = convertDistributionToMarkovLogic(belief.content, getMarkovLogicConstantFromID(belief.id));
		StringBuilder result = new StringBuilder();
		for(MLFormula formula : formulae) {
			result.append(formula.toString());
		}
		return result;
	}
	
	private List<MLFormula> convertDistributionToMarkovLogic(ProbDistribution dist, String belief_id) throws MLException {
		if (dist == null) {
			throw new MLException("no distribution specified");
		}

		if (dist instanceof DistributionWithExistDep) {
			String id;
			if (getMarkovLogicConstantFromID(newInput.id).equals(belief_id)) {
				id = STANDARD_INPUT_ID;
			}
			else {
				id = belief_id;
			}
			return convertDistributionWithExistDep((DistributionWithExistDep) dist, id);
		}
		
		if (dist instanceof CondIndependentDistribs) {
			return convertCondIndependentDistribs((CondIndependentDistribs) dist, belief_id);
		}
		
		if (dist instanceof BasicProbDistribution) {
			return convertBasicProbDistribution((BasicProbDistribution) dist, belief_id);
		}
		else {
			throw new MLException("distribution missing");
		}
	}
	
	private List<MLFormula> convertBasicProbDistribution(BasicProbDistribution dist, String belief_id) throws MLException {
		
		String feature = dist.key;
		
		if(dist.values instanceof FeatureValues) {
			return convertFeatureValuesToMLFormula((FeatureValues) dist.values, belief_id, feature);
		}
		else if(dist.values instanceof NormalValues) {
			return convertNormalValuesToMLFormula((NormalValues) dist.values, belief_id, feature);
		}
		else if(dist.values instanceof FormulaValues) {
			return convertFormulaValuesToMLFormula((FormulaValues) dist.values, belief_id, feature);
		}
		else {
			throw new MLException("distribution missing");
		}
	}
	
	private List<MLFormula> convertFormulaValuesToMLFormula(FormulaValues values, String belief_id, String feature) {
		// TODO Auto-generated method stub
		return null;
	}

	private List<MLFormula> convertNormalValuesToMLFormula(NormalValues values, String belief_id, String feature) {
		// TODO Auto-generated method stub
		return null;
	}

	private List<MLFormula> convertFeatureValuesToMLFormula(FeatureValues values, String belief_id, String feature) throws MLException {
		List<MLFormula> formulae = new LinkedList<MLFormula>();
		
		Set<String> occuring_values = new TreeSet<String>();
		for(FeatureValueProbPair pair : values.values) {
			float weight = convertProbabilityToWeight(pair.prob);
			String value = getFeatureValue(pair.val);
			String formula = setFirstLetterToUppercase(feature) + "(" + belief_id + "," + setFirstLetterToUppercase(value) + ")";
			formulae.add(new MLFormula(weight, formula));
			occuring_values.add(value);
		}
		
		// now we need to add the hard constraint for all the names
		// that have not appeared for the predicate feature
		Set<String> non_occuring_values = new TreeSet<String>(names_for_type.get(setFirstLetterToUppercase(feature)));
		non_occuring_values.remove("None");
		non_occuring_values.removeAll(occuring_values);
		for(String non_occuring_value : non_occuring_values) {
			MLFormula hard_formula = new MLFormula(1f,"!" + setFirstLetterToUppercase(feature) + "(" + belief_id + "," + non_occuring_value + ")");
			hard_formula.setSharp();
			hard_formula.setOpAddImp();
			formulae.add(hard_formula);
		}
		
		// take care of the None case
		MLFormula none_formula = new MLFormula(1f,"!" + setFirstLetterToUppercase(feature) + "(" + belief_id + ",None)");
		none_formula.setSharp();
		none_formula.setOpAddImp();
		formulae.add(none_formula);
		
		return formulae;
	}

	private String getFeatureValue(FeatureValue val) throws MLException {
		if(val instanceof StringValue) {
			return ((StringValue)val).val;
		}
		else if(val instanceof IntegerValue) {
			return new Integer(((IntegerValue)val).val).toString();
		}
		else if(val instanceof BooleanValue) {
			return new Boolean(((BooleanValue)val).val).toString();
		}
		else if(val instanceof UnknownValue) {
			// we should not add anything here
			// because the unknown value is added implicitly during the construction
			// for each type that appearers in the beliefs
			return "Unknown";
		}
		else if(val instanceof PointerValue) {
			// TODO: is this correct to translate the belief id to
			// our internal representation?
			return getMarkovLogicConstantFromID(((PointerValue)val).beliefId.id);
		}

		throw new MLException("unknwon feature value type");
	}

	private List<MLFormula> convertCondIndependentDistribs(
			CondIndependentDistribs dist, String belief_id) throws MLException {
		List<MLFormula> formulae = new LinkedList<MLFormula>();
		
		for (String indepKeys : dist.distribs.keySet()) {
			formulae.addAll(convertDistributionToMarkovLogic(dist.distribs.get(indepKeys), belief_id));
		}
		
		return formulae;
	}
	
	private List<MLFormula> convertDistributionWithExistDep(
			DistributionWithExistDep dist, String belief_id) throws MLException {
		
		ProbDistribution dist_cond = dist.Pc;
		
		float exists_probability = dist.existProb;
		
		List<MLFormula> formulae = new LinkedList<MLFormula>();
		
		Float weight_exists = convertProbabilityToWeight(exists_probability);
		
		String exists = "Existence(" + belief_id + ")";
		MLFormula formula_exists = new MLFormula(weight_exists, exists);
		
		formulae.add(formula_exists);
		
		List<MLFormula> conditional_formulae = convertDistributionToMarkovLogic(dist_cond, belief_id);
		
		// then we have to add formulas for each missing feature and value
		// of type e.g.: Existence(U1) => !Label(U1,Mug).
		Set<String> non_occuring_predicates = new TreeSet<String>(names_for_type.keySet());
		non_occuring_predicates.removeAll(this.predicates_for_belief.get(belief_id));
		String internal_id = setFirstLetterToUppercase(belief_id);
		for(String non_occuring_predicate : non_occuring_predicates) {
			for(String non_occuring_value : names_for_type.get(non_occuring_predicate)) {
				// skip the "None" case
				if(!non_occuring_value.equals("None")) {
					MLFormula hard_formula = new MLFormula(1f, "!" + setFirstLetterToUppercase(non_occuring_predicate) + "(" + internal_id + "," + non_occuring_value + ")");
					hard_formula.setSharp();
					hard_formula.setOpAddImp();
					conditional_formulae.add(hard_formula);
				}
			}
			// handle the "None" case
			MLFormula hard_formula = new MLFormula(1f, setFirstLetterToUppercase(non_occuring_predicate) + "(" + internal_id + ",None)");
			hard_formula.setSharp();
			hard_formula.setOpAddImp();
			conditional_formulae.add(hard_formula);
		}
		
		// for each formula i \in 1..n form dist_cond conditionally dependent
		// on proposition ExistsBeliefK we add a clause:
		// "weight_i Formula <=> ExistsBeliefK"
		for (MLFormula conditional_formula : conditional_formulae) {
			conditional_formula.addThis(exists);
			
			// set the new weight w_new = log(p_exists * p_cond) = log(p_exists * exp(w_old)/(1+exp(w_old)))
			conditional_formula.setWeight(convertConditionalProbabilityToWeight(exists_probability, 
					convertWeightToProbability(conditional_formula.getWeight())));
			formulae.add(conditional_formula);
		}

		return formulae;
	}
	  
	/**
	 * Convert the prior probability into a weight for the Markov Network
	 * according to w = log(p/(1-p))
	 */
	private Float convertProbabilityToWeight(float prob) {
		return new Float(Math.log(prob / (1f - prob)));
	}
	
	private Float convertWeightToProbability(float weight) {
		return new Float(Math.exp(weight)/(1+Math.exp(weight)));
	}
	
	private Float convertConditionalProbabilityToWeight(float prob1, float prob2) {
		return new Float(Math.log(prob1 * prob2));
	}
	
	public static String getMarkovLogicConstantFromID(String id) {
		return "U" + id.replace(":", "_");
	}
	
	public static String getIDFromMarkovLogicConstant (String mlconstant) {
		return mlconstant.substring(1).replace("_", ":");
	}

	private String setFirstLetterToUppercase(String s) {
		return (s.substring(0,1).toUpperCase() + s.substring(1));
	}
	
	private String setFirstLetterToLowercase(String s) {
		return (s.substring(0,1).toLowerCase() + s.substring(1));
	}
	
	private static void log(String s) {
		if (LOGGING) {
			System.out.println("[MLNGenerator] " + s);
		}
	}
	
	@SuppressWarnings("unused")
	private static void debug(final String s) {
		if (DEBUG) {
			System.out.println("[MLNGenerator] " + s);
		}
	}
	
}
