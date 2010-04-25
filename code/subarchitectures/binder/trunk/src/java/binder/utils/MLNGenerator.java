package binder.utils;

import java.util.Collection;
//import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import beliefmodels.autogen.beliefs.Belief;
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

/**
 * 
 * @author Carsten Ehrler (carsten.ehrler@dfki.de)
 *
 * MLNGenerator deals with the serialization of the WM content into Markov Logic 
 */
public class MLNGenerator {

	public static boolean LOGGING = false;

	public static boolean DEBUG = false;

	private static final String EXISTING_UNIONS = "existingunion";

	public static final String STANDARD_INPUT_ID = "P";

	private static final Object NEWLINE = "\n";
	
	private static float EPSILON = 0.00001f;

	MLNPreferences preferences;

	/**
	 * The map keeps track of all the Predicates (keys) and the Names that appear
	 * for each Predicate. It contains at least the two values "None" and "Unknown"
	 * in the Set for each Predicate
	 */
	private Map<String, Set<String>> names_for_type;

	/**
	 * The map keeps track of all the Beliefs (value) that have a Predicate (key)
	 * appear in its distribution. The information is used to model the not-containing
	 * relationship in Markov Logic
	 */
	private Map<String, Set<String>> predicates_for_belief;

	/**
	 * The new Perecpt P that is used to perform inference
	 */
	private Belief newInput;

	/**
	 * Constructor that uses the default MLNPreferences
	 */
	public MLNGenerator() {
		names_for_type = new TreeMap<String, Set<String>>();
		predicates_for_belief = new TreeMap<String, Set<String>>();
		preferences = new MLNPreferences();
	}

	/**
	 * Constructor that uses an external MLNPreference object 
	 * @param preferences used for the serialization into Markov Logic
	 */
	public MLNGenerator(MLNPreferences preferences) {
		names_for_type = new TreeMap<String, Set<String>>();
		predicates_for_belief = new TreeMap<String, Set<String>>();
		this.preferences = preferences;
	}

	/////////////////////////////////////////////////////////////////////////////////////
	// Build the Markov network file
	/////////////////////////////////////////////////////////////////////////////////////

	/**
	 * This method constructs the Markov Logic Network (MLN) in a textual description from the current
	 * state of the WM. The MLN file can then be passed to alchemy to perform inference over possible
	 * bindings of the new Belief b with a set of existing Beliefs (existingUnions)
	 * 
	 * @param b the new belief
	 * @param existingUnions all the existing beliefs that are used to perform inference against b
	 * @param unionsMapping mapping from the existing unions to the new hypothetical outcomes
	 * 			of the binding process
	 * @param newSingleUnionId
	 * 			ID of the new percept
	 * @param MLNFileToWrite
	 * 			Path to the MLN file to which the result is written
	 * @throws MLException
	 */
	public void writeMLNFile(Belief b, Collection<Belief> existingUnions, 
			Map<String,String> unionsMapping, String newSingleUnionId, String MLNFileToWrite) throws MLException {

		newInput = b;
		
		// sanity check of the input
		if(b == null || existingUnions == null || unionsMapping == null || newSingleUnionId == null || MLNFileToWrite == null) {
			throw new MLException("NULL reference in input");
		}
		
		if(preferences == null) {
			throw new MLException("MLNPreferences is NULL");
		}

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

		log(this.names_for_type.toString());
		log(this.predicates_for_belief.toString());

		FileUtils.writeFile(MLNFileToWrite, mln_file.toString());
	}

	/**
	 * Predicates may appear in the correlation specification but it might be the case, that they
	 * are never used in a belief. However, it is necessary to know (have them in the mapping names_for_type)
	 * all these Predicates and their Values because we have to specify them in the constant section
	 * of the MLN file. There are several options to get all the Predicates and Names not used.
	 * 
	 * We decided to use an additional file that specifies all the Predicates and their Values used
	 * in the Correlation specification. The content of the file has the following structure:
	 * 
	 * <PREDICATE> : [<NAME> ]
	 * 
	 * Each line specifies a predicate followed by a colon and a whitespace separated list of all the names 
	 */
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

				names.add("Unknown");
				names.add("None");

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

	/**
	 * Constructs the constant section in the Markov Logic network description
	 * 
	 * @param existingUnions Unions used for the binding process against the new percept
	 * @param unionsMapping Mapping of each belief_id to their internal representation
	 * @param newSingleUnionId Name of the new percept
	 * @return the constant section of the MLN file
	 */
	private StringBuilder constructConstantsSection(Collection<Belief> existingUnions, 
			Map<String,String> unionsMapping, String newSingleUnionId) {
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

	/**
	 * Constructs the predicate section of the MLN file
	 * @return serialized predicate section
	 */
	// 2) predicate section
	private StringBuilder getPredicatesSection() {

		StringBuilder result = new StringBuilder();

		result.append("///////////////////////////////\n");
		result.append("// PREDICATES\n");
		result.append("///////////////////////////////\n");

		result.append(NEWLINE);
		for(String predicate_type : names_for_type.keySet()) {
			result.append(predicate_type + "(belief," + setLettersToLowercase(predicate_type) + "val)\n");
		}

		result.append(NEWLINE);
		result.append("Outcome(outcome)\n");
		result.append("UnifyWith(existingunion)\n");
		result.append("Existence(belief)\n");
		result.append(NEWLINE); 

		return  result;
	}

	/**
	 * Serialize all the content for the existing unions
	 * 
	 * @param existingUnions
	 * @return existing union section in the MLN file
	 * @throws MLException
	 */
	// 3) existing union section
	private StringBuilder constructFormulaeForExistingUnions (Collection<Belief> existingUnions) throws MLException {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// EXISTING UNIONS\n");
		result.append("///////////////////////////////\n");
		result.append(NEWLINE);

		for(Belief belief : existingUnions) {
			if(belief == null) {
				throw new MLException("belief reference is NULL");
			}
			result.append(serializeBelief(belief));
			result.append(NEWLINE);
		}

		return result;
	}

	/**
	 * Serialization of the belief content
	 * @param b Belief which should be serialized
	 * @return serialized percept information section in the MLN file
	 * @throws MLException
	 */
	// 4) new percept section
	private StringBuilder constructFormulaeForPercept (Belief b) throws MLException {
		StringBuilder result = new StringBuilder();
		result.append("///////////////////////////////\n");
		result.append("// NEW PERCEPT\n");
		result.append("///////////////////////////////\n");
		result.append(NEWLINE);
		result.append(serializeBelief(b));
		result.append(NEWLINE);
		return result;
	}

	/**
	 * Constructs the value constraint section for every belief
	 * E.g.:
	 *      !Existence(x) => y=None v !Shape(x,y).
	 *      !Existence(x) => Shape(x,None).
	 * 
	 * @return value constraint section for the MLN file
	 */
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
	/**
	 * Read the correlation file
	 * @return correlation section of the MLN file
	 */
	// 6) correlation section
	private String getCorrelationSection () {
		return FileUtils.readfile(this.preferences.getFile_correlations());
	}
	
	/**
	 * Construct the final outcome section of the MLN file
	 * E.g.:
	 * 		-1.5 Existence(P) => Outcome(U5_3)
	 * 		UnifyWith(U0_3) <=> Outcome(U3_3).
	 * 		UnifyWith(U1_3) <=> Outcome(U4_3).
	 * 		-1.0 Outcome(x)
	 * 		Outcome(x) ^ Outcome(y) => x=y.
	 * 		Exist x Outcome(x).
	 * 
	 * @param existingUnions Unions used for the binding process against the new percept
	 * @param unionsMapping Mapping of each belief_id to their internal representation
	 * @param newSingleUnionId Name of the new percept
	 * @return the outcome section of the MLN file
	 * @throws MLException 
	 */
	// 7) final outcome section
	private StringBuilder extractFinalOutcomeSection(Collection<Belief> existingUnions,
			Map<String,String> unionsMapping, String singleUnionId) throws MLException {
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
			if(new_union == null) {
				throw new MLException("reference is NULL");
			}
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
	
	/**
	 * Serialize all the existing beliefs - which is done by the method {@link #serializeBelief(Belief)}
	 * for each belief in the collection
	 * @param existingUnions the collection of beliefs to serialize in ML
	 * @return the serialized beliefs from the collection
	 */
	private StringBuilder serializeBeliefs(Collection<Belief> existingUnions) {
		StringBuilder result = new StringBuilder();
		Iterator<Belief> iter = existingUnions.iterator();

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
	
	/**
	 * 
	 * @param unionsMapping
	 * @param newSingleUnionId
	 * @return
	 */
	private StringBuilder serializeOutcomes(Map<String, String> unionsMapping, String newSingleUnionId) {
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
	
	/**
	 * 
	 * @return
	 */
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

			result.append("}\n");
		}
		return result;
	}
	
	/**
	 * 
	 * @param distribution
	 * @param predicatesForCurrentBelief
	 * @throws MLException
	 */
	private void extractTypesAndNames(ProbDistribution distribution, Set<String> predicatesForCurrentBelief) throws MLException {
		// TODO: extend with the remaining distributions
		
		if(distribution == null || predicatesForCurrentBelief == null) {
			throw new MLException("reference is NULL");
		}
		
		log("TYPE:");
		log(distribution.toString());

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
				assert subdistribKey != null;
				extractTypesAndNames(((CondIndependentDistribs)distribution).distribs.get(subdistribKey), predicatesForCurrentBelief);
			}
		}
		else {
			log("Distribution unknown: " + distribution.getClass().getCanonicalName());
			throw new MLException ("Error, distribution is null");
		}
	}

	/**
	 * 
	 * @param distrib
	 * @param predicatesForCurrentBelief
	 * @throws MLException 
	 */
	private void extractTypesAndNamesBasicProbDistribution(BasicProbDistribution distrib, Set<String> predicatesForCurrentBelief) throws MLException {
		if(distrib == null) {
			throw new MLException("Dsitribution reference is NULL");
		}
		
		String keyWithUppercase = setFirstLetterToUppercase(distrib.key);
		predicatesForCurrentBelief.add(keyWithUppercase);

		log("ADD PREDICATE");
		log(predicatesForCurrentBelief.toString());

		if(!names_for_type.containsKey(keyWithUppercase)) {
			names_for_type.put(keyWithUppercase, new TreeSet<String>());
		}
		for (String value: getListDistributionValues(distrib.values)) {
			String valWithUppercase = setFirstLetterToUppercase(value);
			names_for_type.get(keyWithUppercase).add(valWithUppercase);
		}
		names_for_type.get(keyWithUppercase).add("Unknown");
		names_for_type.get(keyWithUppercase).add("None");
	}

	/**
	 * 
	 * @param values
	 * @return
	 * @throws MLException 
	 */
	private List<String> getListDistributionValues(DistributionValues values) throws MLException {
		if(values == null) {
			throw new MLException("DistributionValues is null");
		}
		
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
	
	/**
	 * 
	 * @param values
	 * @return
	 * @throws MLException 
	 */
	private List<String> namesAndTypesFormulaValues(FormulaValues values) throws MLException {
		if(values == null) {
			throw new MLException("FormulaValues is null");
		}
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * 
	 * @param values
	 * @return
	 * @throws MLException 
	 */
	private List<String> namesAndTypesNormalValues(NormalValues values) throws MLException {
		if(values == null) {
			throw new MLException("NormalValues is null");
		}
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * 
	 * @param values
	 * @return
	 * @throws MLException 
	 */
	private List<String> namesAndTypesFeatureValues(FeatureValues values) throws MLException {
		if(values == null) {
			throw new MLException("FeatureValues is null");
		}
		
		List<String> result = new LinkedList<String>();

		for(FeatureValueProbPair feature_pair : values.values) {
			FeatureValue val = feature_pair.val;
			if(val instanceof StringValue) {
				result.add("v"+((StringValue)val).val.replace(":", "_"));
				
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
	
	/**
	 * 
	 * @param belief
	 * @return
	 * @throws MLException
	 */
	private StringBuilder serializeBelief(Belief belief) throws MLException {
		if(belief == null) {
			throw new MLException("Belief is null");
		}
		
		String belief_id;

		// convert to internal ids for the name
		if(belief.id.equals(newInput.id)) {
			belief_id = STANDARD_INPUT_ID;
		}
		else {
			belief_id = getMarkovLogicConstantFromID(belief.id);
		}

		List<MLFormula> formulae = convertDistributionToMarkovLogic(belief.content, belief_id);
		StringBuilder result = new StringBuilder();
		for(MLFormula formula : formulae) {
			result.append(formula.toString());
		}
		return result;
	}
	
	/**
	 * 
	 * @param dist
	 * @param belief_id
	 * @return
	 * @throws MLException
	 */
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
	
	/**
	 * 
	 * @param dist
	 * @param belief_id
	 * @return
	 * @throws MLException
	 */
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
	
	/**
	 * 
	 * @param values
	 * @param belief_id
	 * @param feature
	 * @return
	 */
	private List<MLFormula> convertFormulaValuesToMLFormula(FormulaValues values, String belief_id, String feature) {
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * 
	 * @param values
	 * @param belief_id
	 * @param feature
	 * @return
	 */
	private List<MLFormula> convertNormalValuesToMLFormula(NormalValues values, String belief_id, String feature) {
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * 
	 * @param values
	 * @param belief_id
	 * @param feature
	 * @return
	 * @throws MLException
	 */
	private List<MLFormula> convertFeatureValuesToMLFormula(FeatureValues values, String belief_id, String feature) throws MLException {
		List<MLFormula> formulae = new LinkedList<MLFormula>();

		Set<String> occuring_values = new TreeSet<String>();
		for(FeatureValueProbPair pair : values.values) {
			try {
			float weight = convertProbabilityToWeight(pair.prob);
			String value = getFeatureValue(pair.val);
			String formula = setFirstLetterToUppercase(feature) + "(" + belief_id + "," + setFirstLetterToUppercase(value) + ")";
			formulae.add(new MLFormula(weight, formula));
			occuring_values.add(setFirstLetterToUppercase(value));
			}
			catch (MLException e) { }
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
	
	/**
	 * 
	 * @param val
	 * @return
	 * @throws MLException
	 */
	private String getFeatureValue(FeatureValue val) throws MLException {
		if(val instanceof StringValue) {
			return "v"+((StringValue)val).val.replace(":", "_");
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
	
	/**
	 * 
	 * @param dist
	 * @param belief_id
	 * @return
	 * @throws MLException
	 */
	private List<MLFormula> convertCondIndependentDistribs(
			CondIndependentDistribs dist, String belief_id) throws MLException {
		List<MLFormula> formulae = new LinkedList<MLFormula>();

		for (String indepKeys : dist.distribs.keySet()) {
			formulae.addAll(convertDistributionToMarkovLogic(dist.distribs.get(indepKeys), belief_id));
		}

		return formulae;
	}
	
	/**
	 * 
	 * @param dist
	 * @param belief_id
	 * @return
	 * @throws MLException
	 */
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
		non_occuring_predicates.removeAll(predicates_for_belief.get(belief_id));
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
	 * 
	 * @paramm prob The probability to convert
	 * @return the weight in Markov Logic
	 * @throws MLException 
	 */
	
	private Float convertProbabilityToWeight(float prob) throws MLException {
		if(Float.compare(prob, 1f) > 0 || Float.compare(prob, 0f) < 0) {
			throw new MLException("Value is not a probability: " + prob);
		}
		// handle the border case, where denominator approaches 0 or 1
		if(Float.compare(prob, 1f) >= 0) {
			return Float.MAX_VALUE;
		}
		if(Float.compare(prob, 0f) <= 0) {
			return Float.MIN_VALUE;
		}
		else {
			return new Float(Math.log(prob / (1f - prob)));
		}
	}

	/**
	 * Convert a weight baqck to a probability according to
	 * p = exp(w)/(1+exp(w))
	 * 
	 * @param weight
	 * @return the corresponding probability
	 * @throws MLException 
	 */
	private Float convertWeightToProbability(float weight) throws MLException {
		// handle the border cases
		if(Float.compare(weight, Float.MIN_VALUE) == 0) {
			return 0f;
		}
		if(Float.compare(weight, Float.MAX_VALUE) == 0) {
			return 1f;
		}
		
		float a = (float)Math.exp(weight);
		
		a = a/(a+1);
		
		if(Float.compare(a, Float.NaN) == 0) {
			throw new MLException("Error in probability conversion...");
		}
		
		return a;
	}
	
	/**
	 * Calculate the weight for a Markov Logic formula in an exists
	 * relationship. w = log(prob1 * prob2) = log(prob1) + log(prob2)
	 * 
	 * @param prob1 probability of the Exists
	 * @param prob2 probability of the conditioned predicate
	 * @return weight of the implication Exists => Predicate
	 * @throws MLException 
	 */
	private Float convertConditionalProbabilityToWeight(float prob1, float prob2) throws MLException {
		if(prob1 > 1f || prob1 < 0 || prob2 > 1f || prob2 < 0) {
			throw new MLException("Value is not a probability");
		}
		return new Float(Math.log(prob1) + Math.log(prob2));
	}
	
	/**
	 * 
	 * @param id
	 * @return
	 */
	public static String getMarkovLogicConstantFromID(String id) {
		return "U" + id.replace(":", "_");
	}
	
	/**
	 * 
	 * @param mlconstant
	 * @return
	 */
	public static String getIDFromMarkovLogicConstant (String mlconstant) {
		return mlconstant.substring(1).replace("_", ":");
	}
	
	/**
	 * 
	 * @param s
	 * @return
	 */
	private String setFirstLetterToUppercase(String s) {
		return (s.substring(0,1).toUpperCase() + s.substring(1).toLowerCase());
	}
	
	/**
	 * 
	 * @param s
	 * @return
	 */
	private String setLettersToLowercase(String s) {
		return (s.toLowerCase());
	}
	
	/**
	 * 
	 * @param s
	 */
	private static void log(String s) {
		if (LOGGING) {
			System.out.println("[MLNGenerator] " + s);
		}
	}
	
	/**
	 * 
	 * @param s
	 */
	@SuppressWarnings("unused")
	private static void debug(final String s) {
		if (DEBUG) {
			System.out.println("[MLNGenerator] " + s);
		}
	}

}
