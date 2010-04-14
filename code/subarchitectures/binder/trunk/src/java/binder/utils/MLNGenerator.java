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
import beliefmodels.autogen.featurecontent.MemoryAddressValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.featurecontent.StringValue;
import beliefmodels.autogen.featurecontent.UnknownValue;
import binder.ml.MLException;
import binder.ml.MLFormula;
import binder.ml.Predicate;

public class MLNGenerator {

	public static boolean LOGGING = true;
	
	public static boolean DEBUG = true;
	
	private static final String EXISTING_UNIONS = "existingunion";

	private static final Object NEWLINE = "\n";

	public static String markovlogicDir = "subarchitectures/binder/markovlogic/";
	
	private Map<String, Set<String>> names_for_type;
	
	public MLNGenerator() {
		names_for_type = new TreeMap<String, Set<String>>();
	}
	
	/////////////////////////////////////////////////////////////////////////////////////
	// Build the Markov network file
	/////////////////////////////////////////////////////////////////////////////////////
	
	// we build the corresponding Markov logic network file step by step
	public void writeMLNFile(PerceptBelief b, Collection<PerceptUnionBelief> existingUnions, HashMap<String,String> unionsMapping, String MLNFileToWrite) throws MLException {
		StringBuilder mln_file = new StringBuilder();
		
		// before we can start with the serialization we need to know all the names
		// and types that appear in the beliefs
		extractTypesAndNames(b.content);
		for(Belief belief : existingUnions) {
			extractTypesAndNames(belief.content);
		}
		
		// now lets do the serialization
		mln_file.append(constructConstantsSection(existingUnions, unionsMapping));
		mln_file.append(getPredicatesSection());
		mln_file.append(constructFormulaeForExistingUnions(existingUnions));
		mln_file.append(constructFormulaeForPercept(b));
		mln_file.append(getFeatValueConstraintsSection());
		mln_file.append(getCorrelationSection());
		mln_file.append(extractFinalOutcomeSection(existingUnions, unionsMapping));
		
		FileUtils.writeFile(MLNFileToWrite, mln_file.toString());
	}
	
	// 1) constants section 
	private StringBuilder constructConstantsSection (Collection<PerceptUnionBelief> existingUnions, HashMap<String,String> unionsMapping) {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// CONSTANTS\n");
		result.append("///////////////////////////////\n");
		
		// 1.1) beliefs
		result.append(NEWLINE);
		result.append("// beliefs");
		result.append(serializeBeliefs(existingUnions));
		
		// 1.2) possible new outcomes
		result.append(NEWLINE);
		result.append("// possible outcomes of the grouping process");
		result.append(serializeOutcomes(unionsMapping));
		
		// 1.3) feature values
		result.append(NEWLINE);
		result.append("// feature values");
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
			result.append(predicate_type + "(belief," + names_for_type.get(predicate_type) + "val)\n");
		}
		
		result.append(NEWLINE);
		result.append("Outcome(outcome)\n");
		result.append("UnifyWith(existingunion)\n");
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
			result.append("Shape(x,y) ^ " + predicate_type + "(x,z) => y=z.\n");
			result.append("EXIST y " + predicate_type + "(x,y).\n");
		}
		result.append(NEWLINE);
		
		return result;
	}
	
	// 6) correlation section
	private String getCorrelationSection () {
		// TODO: @Pierre the correlations have to come from somewhere else of course.
		// However, there is a problem because we can't assume that all the names that
		// appear in the correlations have been defines already in the constants section.
		// We need to resolve this problem somehow either by making all names static
		// i.e. known a prior or we parse the correlation file...
		// FIXME: we need to adapt the method
		// FIXME: implement path to the file in the constructor
		
		return FileUtils.readfile(markovlogicDir + "grouping/correlations.mln");
	}
	
	// 7) final outcome section
	private StringBuilder extractFinalOutcomeSection(Collection<PerceptUnionBelief> existingUnions, HashMap<String,String> unionsMapping) {
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
		assert unionsMapping.containsKey("P");
		result.append("-1.5 Existence(P) => Outcome(" + getMarkovLogicConstantFromID(unionsMapping.get("P")) + ")\n");
		
		// then comes the rest of the possible results
		Set<String> old_unions = new TreeSet<String>(unionsMapping.keySet());
		assert old_unions.remove("P");
		for(String old_union : old_unions) {
			result.append("UnifyWith(" + getMarkovLogicConstantFromID(old_union) + 
					") <=> Outcome(" + getMarkovLogicConstantFromID(unionsMapping.get(old_union)) + ").");
		}
		
		// b) mutual exclusivity and unicity
		result.append("-1 Outcome(x)\n");
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
	
	private StringBuilder serializeOutcomes(HashMap<String, String> unionsMapping) {
		StringBuilder result = new StringBuilder();
		result.append("outcome = {");
		
		Iterator<String> iter = unionsMapping.values().iterator();
		result.append(getMarkovLogicConstantFromID(iter.next()));
		while(iter.hasNext()) {
			result.append("," + getMarkovLogicConstantFromID(iter.next()));
		}
		
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
	
	private void extractTypesAndNames(ProbDistribution distribution) {
		// TODO: extend with the remaining distributions
		
		if(distribution instanceof BasicProbDistribution) {
			extractTypesAndNamesBasicProbDistribution((BasicProbDistribution) distribution);
			return;
		}
		
		else if (distribution instanceof DistributionWithExistDep) {
			extractTypesAndNames (((DistributionWithExistDep)distribution).Pc);
		}
		
		log("Distribution unknown: " + distribution.getClass().getCanonicalName());
	}
	
	private void extractTypesAndNamesBasicProbDistribution(BasicProbDistribution belief) {
		if(!names_for_type.containsKey(belief.key)) {
			names_for_type.put(belief.key, new TreeSet<String>());
		}
		names_for_type.get(belief.key).addAll(addDistributionValuesToType(belief.values));
	}

	private List<String> addDistributionValuesToType(DistributionValues values) {
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
				result.add(getMarkovLogicConstantFromID(((PointerValue)val).beliefId));
				continue;
			}
			if(val instanceof MemoryAddressValue) {
				// FIXME: what to do here with the WM memory pointer???
				result.add(((MemoryAddressValue)val).beliefId.toString());
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
			return convertDistributionWithExistDep((DistributionWithExistDep) dist, belief_id);
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
			String formula = feature + "(" + belief_id + "," + value + ")";
			formulae.add(new MLFormula(weight, formula));
			occuring_values.add(value);
		}
		
		// now we need to add the hard constraint for all the names
		// that have not appeared for the predicate feature
		Set<String> non_occuring_values = new TreeSet<String>(names_for_type.get(feature));
		non_occuring_values.remove("None");
		non_occuring_values.removeAll(occuring_values);
		for(String non_occuring_value : non_occuring_values) {
			MLFormula hard_formula = new MLFormula(1f,"!" + feature + "(" + belief_id + "," + non_occuring_value + ")");
			hard_formula.setSharp();
			formulae.add(hard_formula);
		}
		
		// take care of the None case
		MLFormula none_formula = new MLFormula(1f, feature + "(" + belief_id + ",None)");
		none_formula.setSharp();
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
			return getMarkovLogicConstantFromID(((PointerValue)val).beliefId);
		}
		else if(val instanceof MemoryAddressValue) {
			// FIXME: what to do here with the WM memory pointer???
			return ((MemoryAddressValue)val).beliefId.toString();
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

		// for each formula i \in 1..n form dist_cond conditionally dependent
		// on proposition ExistsBeliefK we add a clause:
		// "weight_i Formula <=> ExistsBeliefK"
		for (MLFormula conditional_formula : conditional_formulae) {
			conditional_formula.addThisImpliesFormula(exists);
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
	
	public static String getMarkovLogicConstantFromID(String id) {
		return "U" + id.replace(":", "_");
	}
	
	public  static String getIDFromMarkovLogicSontant (String mlconstant) {
		return mlconstant.substring(1).replace("_", ":");
	}


	private static void log(String s) {
		if (LOGGING) {
			System.out.println("[MLNGenerator] " + s);
		}
	}

	private static void debug(String s) {
		if (DEBUG) {
			System.out.println("[MLNGenerator] " + s);
		}
	}
	
}
