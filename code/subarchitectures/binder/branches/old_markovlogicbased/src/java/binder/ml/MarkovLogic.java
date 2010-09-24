package binder.ml;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.FormulaProbPair;
import beliefmodels.autogen.distribs.FormulaValues;
import beliefmodels.autogen.distribs.NormalValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.*;
import beliefmodels.autogen.logicalcontent.BinaryOp;
import beliefmodels.autogen.logicalcontent.ComplexFormula;
import beliefmodels.autogen.logicalcontent.ElementaryFormula;
import beliefmodels.autogen.logicalcontent.Formula;
import beliefmodels.autogen.logicalcontent.ModalFormula;
import beliefmodels.autogen.logicalcontent.NegatedFormula;
import beliefmodels.autogen.logicalcontent.PointerFormula;


/**
 * 
 * @author Carsten Ehrler (carsten.ehrler@dfki.de)
 * 
 */
public class MarkovLogic {

	private static final String TO_INFER = "to_infer";
	
	private static final String BELIEF_TYPE = "belief_id";

	private static final String MLN_SPACER = "\n//////////////////////////////////////////////\n\n";

	private static final float DELTA = 0.001f;

	private static long next_internal_belief_id;

	private BiMap<String, Long> belief_id_to_internal_id;

	/**
	 * This map maps each belief to its Markov Logic representation
	 */
	private Map<String, List<MLFormula>> beliefs_it_to_mlformula;
	private Map<String, Belief> belief_id_to_belief;

	private PredicateData predicate_data;

	/**
	 * Here we keep track of each sub-type of belief e.g. Percept,
	 * PerceptUnion,...
	 */
	private Map<String, Set<Belief>> types_of_beliefs;

	private MLPreferences preferences;

	private static String INFER_PERCEPTUAL_GROUPING = "Group";
	@SuppressWarnings("unused")
	private static String INFER_MULTI_MODAL_FUSION = "Fusion";
	@SuppressWarnings("unused")
	private static String INFER_TRACKING = "Track";
	@SuppressWarnings("unused")
	private static String INFER_TEMPORAL_SMOOTHING = "Smooth";

	// disjunction and conjunction operator for the complex formula
	private static String OP_DISJ = "v";
	private static String OP_CONJ = "^";
	private static String OP_NEG = "!";
	// braces for formula composition
	private static String LBRACE = " (";
	private static String RBRACE = ") ";

	// braces and commas for functions
	private static String LFUNC = "(";
	private static String RFUNC = ")";
	private static String CFUNC = ", ";

	// suffix that is prepended to each belief_id in belief pointer reference
	private static String PNTR_PREFIX = "Belief";

	private static String VAL_UNKNOWN = "Unknown";

	public MarkovLogic() {
		beliefs_it_to_mlformula = new TreeMap<String, List<MLFormula>>();
		types_of_beliefs = new TreeMap<String, Set<Belief>>();
		predicate_data = new PredicateData();
		belief_id_to_internal_id = new BiMap<String, Long>();
		belief_id_to_belief = new TreeMap<String, Belief>();
	}

	public void addBelief(Belief belief) throws MLException {
		if (belief == null) {
			throw new MLException("Reference is null");
		}

		if (beliefs_it_to_mlformula.containsKey(belief.id)) {
			throw new MLException("Belief with same ID already exists");
		}

		belief_id_to_belief.put(belief.id, belief);
		belief_id_to_internal_id.put(belief.id, next_internal_belief_id);
		next_internal_belief_id++;

		beliefs_it_to_mlformula.put(belief.id,
				convertDistributionToMarkovLogic(belief.content, belief.id));

		String type = belief.getClass().getCanonicalName();

		if (types_of_beliefs.containsKey(type)) {
			types_of_beliefs.get(type).add(belief);
		} else {
			Set<Belief> new_id = new HashSet<Belief>();
			new_id.add(belief);
			types_of_beliefs.put(type, new_id);
		}
	}

	public String addBeliefTEST(Belief belief) throws MLException {
		addBelief(belief);
		List<MLFormula> ml = beliefs_it_to_mlformula.get(belief.id);

		StringBuilder serialized = new StringBuilder();

		for (MLFormula s : ml) {
			serialized.append(s + "\n");
		}

		return serialized.toString();
	}

	/**
	 * Converts the set of Beliefs to their representation with internal ids and
	 * formates the set of names for the Markov Logic Network
	 * 
	 * output is something like: to_infer = {Belief1 , Belief2, ... , BeliefN}
	 * 
	 * @param namesOfBeliefsToInfer
	 * @return
	 */
	private StringBuilder beliefsToInfer(Set<Belief> namesOfBeliefsToInfer) {
		StringBuilder infer = new StringBuilder();
		infer.append(TO_INFER + " = { ");

		Iterator<Belief> iter = namesOfBeliefsToInfer.iterator();

		infer.append(getInternalName((iter.next()).id));

		while (iter.hasNext()) {
			infer.append(" , ");
			infer.append(getInternalName((iter.next()).id));
		}
		infer.append(" }\n");
		return infer;
	}

	private String buildFormulaFromFeatureValue(String feature, String value,
			String belief_id) {
		return feature + LFUNC + getInternalName(belief_id) + CFUNC + value + RFUNC;
	}

	private StringBuilder buildMarkovLogicNetwork() throws MLException {
		StringBuilder mln = new StringBuilder();
		String types = predicate_data.getTypes();
		String declarations = predicate_data.getDeclarations();
		String correlations = getCorrelations();
		String binder_content = getBinderContent();

		mln.append(types);
		mln.append(MLN_SPACER);
		mln.append(declarations);
		mln.append(MLN_SPACER);
		mln.append(correlations);
		mln.append(MLN_SPACER);
		mln.append(binder_content);
		mln.append(MLN_SPACER);
		return mln;
	}

	private String convertComplexFormula(ComplexFormula formula,
			String belief_id) throws MLException {
		StringBuilder complex_formula = new StringBuilder();

		String OP = null;
		if (formula.op == BinaryOp.conj) {
			OP = OP_CONJ;
		} else if (formula.op == BinaryOp.disj) {
			OP = OP_DISJ;
		} else {
			throw new MLException("unknwon binary operator in BinaryOP enum");
		}

		// we add the first formula to the string builder
		// to get the operators OP working easily
		assert formula.forms.size() >= 1;
		String first = convertFormulaToString(formula.forms.get(0), belief_id);
		complex_formula.append(LBRACE);
		complex_formula.append(first);
		complex_formula.append(RBRACE);

		// and then the (possibly empty) rest of the complex formula
		for (int i = 1; i < formula.forms.size(); ++i) {
			complex_formula.append(OP);
			complex_formula.append(LBRACE);
			complex_formula.append(convertFormulaToString(formula.forms.get(0),
					belief_id));
			complex_formula.append(RBRACE);
		}
		return complex_formula.toString();
	}

	// /////////////////////////////////////////////////////////////////////////////////////
	//
	// Methods which work on PropDistributions come here
	//
	// /////////////////////////////////////////////////////////////////////////////////////

	private List<MLFormula> convertCondIndependentDistribs(
			CondIndependentDistribs dist, String belief_id) throws MLException {
		List<MLFormula> formulae = new LinkedList<MLFormula>();

		for (String indepKeys : dist.distribs.keySet()) {
			formulae.addAll(convertDistributionToMarkovLogic(dist.distribs.get(indepKeys), belief_id));
		}

		return formulae;
	}

	private List<MLFormula> convertFormulaDistribution(
			BasicProbDistribution dist, String belief_id) throws MLException {
		List<MLFormula> formulae = new LinkedList<MLFormula>();

		if (dist.values instanceof FormulaValues) {
		for (FormulaProbPair pair : ((FormulaValues)dist.values).pairs) {
			Float weight = convertProbabilityToWeight(pair.prob);
			String formula = convertFormulaToString(pair.form, belief_id);
			formulae.add(new MLFormula(weight, formula));
		}
		}
		else {
			throw new MLException("error, dist.values are of wrong type");
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
	private List<MLFormula> convertDistributionToMarkovLogic(
			ProbDistribution dist, String belief_id) throws MLException {
		if (dist == null) {
			throw new MLException("no distribution specified");
		}

		if (dist instanceof DistributionWithExistDep) {
			return convertDistributionWithExistDep(
					(DistributionWithExistDep) dist, belief_id);
		}
		
		if (dist instanceof CondIndependentDistribs) {
			return convertCondIndependentDistribs(
					(CondIndependentDistribs) dist, belief_id);
		}
		
		if (dist instanceof BasicProbDistribution) {
			if (((BasicProbDistribution)dist).values instanceof FeatureValues) {
				return convertFeatureDistribution(
						(BasicProbDistribution) dist, belief_id);
			}
			else if (((BasicProbDistribution)dist).values instanceof FormulaValues) {
				return convertFormulaDistribution(
						(BasicProbDistribution) dist, belief_id);
			}
			else if (((BasicProbDistribution)dist).values instanceof NormalValues) {
				return convertNormalDistribution(
						(BasicProbDistribution) dist, belief_id);
			}
			else {
				throw new MLException("distribution missing");
			}
			
		}
	
		 else {
			throw new MLException("distribution missing");
		}
	}

	
	private List<MLFormula> convertDistributionWithExistDep(
			DistributionWithExistDep dist, String belief_id) throws MLException {

		ProbDistribution dist_cond = dist.Pc;

		float exists_probability = dist.existProb;

		List<MLFormula> formulae = new LinkedList<MLFormula>();

		Float weight_exists = convertProbabilityToWeight(exists_probability);
		Float weight_not_exists = convertProbabilityToWeight(1f - exists_probability);

		String exists = "Exists(" + belief_id + ")";
		MLFormula formula_exists = new MLFormula(weight_exists, exists);
		MLFormula formula_not_exists = new MLFormula(weight_not_exists, "!"
				+ exists);

		formulae.add(formula_exists);
		formulae.add(formula_not_exists);

		List<MLFormula> conditional_formulae = convertDistributionToMarkovLogic(
				dist_cond, belief_id);

		// for each formula i \in 1..n form dist_cond conditionally dependent
		// on proposition ExistsBeliefK we add a clause:
		// "weight_i Formula <=> ExistsBeliefK"
		for (MLFormula conditional_formula : conditional_formulae) {
			//conditional_formula.addThisEquivalence(exists);
			formulae.add(conditional_formula);
		}

		return formulae;
	}

	private List<MLFormula> convertFeatureDistribution(BasicProbDistribution dist, String belief_id) throws MLException {
	
		// BIG FAT WARNING: we currently don't have access to the feature label anymore!!
	//	String feature = dist.feat.toString();
		String feature = "blabla";
		
		List<MLFormula> formulae = new LinkedList<MLFormula>();

		Predicate predicate_from_feature = new Predicate(feature, feature
				.toLowerCase());

		if (predicate_data.hasPredicate(feature)) {
			predicate_data.addPredicateToBelief(belief_id,
					predicate_from_feature);
		} else {
			predicate_data.addNewPredictate(belief_id, predicate_from_feature);
		}
		
		predicate_data.addValueForType(BELIEF_TYPE, getInternalName(belief_id));
		
		
		if (dist.values instanceof FeatureValues) {
		for (FeatureValueProbPair feature_value_pair : ((FeatureValues)dist.values).values) {
			MLFormula ml_formula = convertFeatureValuePairToMLFormula(
					feature_value_pair, feature, belief_id);
			formulae.add(ml_formula);
		}
		}
		else {
			throw new MLException("error, distribution values of wrong type");
		}
		return formulae;
	}

	private MLFormula convertFeatureValuePairToMLFormula(
			FeatureValueProbPair feature_value_pair, String feature,
			String belief_id) throws MLException {
		// FIXME: pointer value should be converted to internal presentation
		// FIXME: handling of integers is not correct
		// FIXME: look at VAL_UNKNOWN and think about it
		
		Float weight = convertProbabilityToWeight(feature_value_pair.prob);
		FeatureValue feature_value = feature_value_pair.val;
		String value = "";
		if (feature_value instanceof StringValue) {
			value = ((StringValue) feature_value).val;
		} else if (feature_value instanceof IntegerValue) {
			value = new Integer(((IntegerValue) feature_value).val).toString();
		} else if (feature_value instanceof BooleanValue) {
			value = new Boolean(((BooleanValue) feature_value).val).toString();
		} else if (feature_value instanceof UnknownValue) {
			value = VAL_UNKNOWN;
		} else if (feature_value instanceof PointerValue) {
			value = getInternalName(((PointerValue) feature_value).beliefId.id);
		}

		predicate_data.addValueForType(feature.toLowerCase(), value);

		String formula = buildFormulaFromFeatureValue(feature, value, belief_id);
		return new MLFormula(weight, formula);
	}

	/**
	 * This method converts a formula into a textual description
	 * 
	 * @param formula
	 *            the formula that should be converted
	 * @param belief_id
	 *            the id of the belief that contains this formula. this id is
	 *            crucial for the conversion of modal operators into a
	 *            propositional form e.g.: <Colour>Red => Colour(Belief1,Red)
	 * @return the string conversion of the formula
	 * @throws MLException
	 */
	private String convertFormulaToString(Formula formula, String belief_id)
			throws MLException {
		assert formula != null;

		// a formula can be one of several subclasses we need to distinguish
		// ElementaryFormula
		if (formula instanceof ElementaryFormula) {
			return ((ElementaryFormula) formula).prop;
		}

		// PointerFormula
		if (formula instanceof PointerFormula) {
			PointerFormula pointer = (PointerFormula) formula;

			if (!belief_id_to_internal_id.containsKey(pointer.beliefPointer)) {
				throw new MLException(
						"Belief pointer references unknown belief");
			}

			return getInternalName(((PointerFormula) formula).beliefPointer);
		}

		// NegatedFormula
		if (formula instanceof NegatedFormula) {
			String to_negate = convertFormulaToString(
					((NegatedFormula) formula).negForm, belief_id);
			return OP_NEG + LBRACE + to_negate + RBRACE;
		}

		// ModalFormula
		if (formula instanceof ModalFormula) {
			return convertModalFormula((ModalFormula) formula, belief_id);
		}

		// ComplexFormula
		if (formula instanceof ComplexFormula) {
			return convertComplexFormula((ComplexFormula) formula, belief_id);
		}

		// Seems like the formula is not implemented or something is wrong
		throw new MLException("Formula type unknown");
	}

	private String convertModalFormula(ModalFormula formula, String belief_id)
			throws MLException {
		String modal_op = formula.op;
		String argument = convertFormulaToString(formula.form, belief_id);

		// if the argument formula.form is of type ElementaryFormula
		// then we have to update this.names_for_predicates as we have
		// found a possibly new name of type feature
		Predicate predicate;
		String arg_type = null;
		if (formula.form instanceof ElementaryFormula) {
			arg_type = modal_op.toLowerCase();

		}

		predicate = new Predicate(modal_op, arg_type);

		if (predicate_data.existsPredicate(predicate)) {
			predicate_data.addPredicateToBelief(belief_id, predicate);
		} else {
			predicate_data.addNewPredictate(belief_id, predicate);
		}

		predicate_data.addValueForType(BELIEF_TYPE, getInternalName(belief_id));

		if (arg_type != null) {
			predicate_data.addValueForType(arg_type, argument);
		}

		return modal_op + getInternalName(belief_id) + CFUNC + argument + RFUNC;
	}

	private List<MLFormula> convertNormalDistribution(BasicProbDistribution dist,
			String belief_id) throws MLException {
		// TODO Auto-generated method stub
		throw new MLException("Normal distribution not yet implemented");
	}

	// /////////////////////////////////////////////////////////////////////////////////////
	//
	// Methods to convert a formulae to string representations go here
	//
	// /////////////////////////////////////////////////////////////////////////////////////

	/**
	 * Convert the prior probability into a weight for the Markov Network
	 * according to w = log(p/(1-p))
	 */
	private Float convertProbabilityToWeight(float prob) {
		return new Float(Math.log(prob / (1f - prob)));
	}
	private String getBinderContent() {
		StringBuilder binder_content = new StringBuilder();

		for (String belief_id : beliefs_it_to_mlformula.keySet()) {
			List<MLFormula> ml_formulae = beliefs_it_to_mlformula
					.get(belief_id);
			binder_content.append("// Belief: ");
			binder_content.append(belief_id);
			binder_content.append("\n");
			for (MLFormula ml_formula : ml_formulae) {
				binder_content.append(ml_formula.toString());
			}
		}
		return binder_content.toString();
	}
	private String[] getCommand(String query) {
		List<String> command = new LinkedList<String>();
		// path to alchemy
		command.add(preferences.getPath_to_alchemy());
		// the mln file
		command.add(" -i " + preferences.getPath_to_input());
		// the result
		command.add(" -r " + preferences.getPath_to_output());
		// the empty database
		command.add(" -e " + preferences.getPath_to_db());
		// the query
		command.add(" -q " + query);
		// inference parameters
		for (String param : preferences.getParameters()) {
			command.add(param);
		}
		return command.toArray(new String[command.size()]);
	}

	private String getCorrelations() throws MLException {
		BufferedReader buffered_reader = null;
		try {
			FileReader file_reader = new FileReader(preferences.getPath_to_correlations());
			buffered_reader = new BufferedReader(file_reader);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		StringBuilder correlations = new StringBuilder();
		try {
			while(buffered_reader.ready()) {
				correlations.append(buffered_reader.readLine());
				correlations.append("\n");
			}
		} catch(Exception e) {
			throw new MLException("error while reading correlations" + e.getStackTrace());
		}
		return correlations.toString();
	}
	
	
	/**
	 * This method computes the probability of the Exists and checks that
	 * Distribution dist_exists is of type DiscriteDistribution with exactly two
	 * values for Exists and !Exists
	 * 
	 * @param dist_exists
	 *            The distribution specified for the Exists
	 * 
	 * @return returns the probability of P(Exists=true) where P(Exists=false) =
	 *         1 - P(Exists=true)
	 * 
	 * @throws MLException
	 *             MLException is thrown if the distribution dist_exists is not
	 *             of type DiscreteDistribution with exactly two pairs <true,p1>
	 *             and <false, p2 such that p1 + p2 = 1
	 */
	/**private float getExistsProbability(ProbDistribution dist_exists)
			throws MLException {
		if (!(dist_exists instanceof Basi)) {
			throw new MLException(
					"Exists distribution must have type DiscreteDistribution!");
		}

		DiscreteDistribution dist = (DiscreteDistribution) dist_exists;

		if (dist.pairs.size() != 2) {
			throw new MLException(
					"DiscreteDistribution must have exactly two arguments");
		} 

		if (!(dist.pairs.get(0).form instanceof ElementaryFormula)) {
			throw new MLException(
					"First pair formula must be of type ElementaryFormula");
		}
 
		if (!(dist.pairs.get(1).form instanceof ElementaryFormula)) {
			throw new MLException(
					"Second pair formula must be of type ElementaryFormula");
		}

		ElementaryFormula formula0 = (ElementaryFormula) dist.pairs.get(0).form;
		ElementaryFormula formula1 = (ElementaryFormula) dist.pairs.get(1).form;

		float prob0 = dist.pairs.get(0).prob;
		float prob1 = dist.pairs.get(1).prob;

		if (prob0 + prob1 <= 1f + DELTA && prob0 + prob1 >= 1f + DELTA) {
			throw new MLException(
					"Probabilities must sum to one, current value is: "
							+ (new Float(prob0 + prob1)).toString());
		}

		if (!formula0.prop.equalsIgnoreCase("true")
				|| !formula0.prop.equalsIgnoreCase("false")) {
			throw new MLException(
					"first proposition must be either true or false");
		}

		if (!formula1.prop.equalsIgnoreCase("true")
				|| !formula1.prop.equalsIgnoreCase("false")) {
			throw new MLException(
					"second proposition must be either true or false");
		}

		if (formula0.prop.equalsIgnoreCase("true")) {
			return prob0;
		} else {
			return prob1;
		}
	} */
	
	private String getInternalName(String belief_id) {
		return PNTR_PREFIX + belief_id_to_internal_id.get(belief_id).toString();
	}
	public Map<Belief, Float> infer(Belief belief) throws MLException {

		if (belief == null) {
			throw new MLException("reference to belief is null!");
		}
		
		// construct the set of beliefs currently in the WM which should be used
		// for inference with the new belief. basically, this includes
		// all beliefs of the same type that are currently added to this ML object 
		Set<Belief> beliefs_to_infer_with = new HashSet<Belief>(types_of_beliefs.get(belief.getClass().getCanonicalName()));
		
		// then we add the new belief temporarily to the set of beliefs
		// it will be removed again after inference is complete (see method end)
		addBelief(belief);
		
		// construct the Markov logic network
		StringBuilder mln = new StringBuilder();
		mln.append(beliefsToInfer(beliefs_to_infer_with));
		mln.append(buildMarkovLogicNetwork());
		
		Map<Belief, Float> result = null;
		
		// we have to distinguish all different types of beliefs
		// and then decide which kind of inference we have to perform
		
		// perceptual grouping
		if (belief instanceof PerceptBelief) {
			result = inferPerceptBelief((PerceptBelief) belief, mln);
		}
		
		// multi modal fusion
		else if (belief instanceof PerceptUnionBelief) {
			result =  inferPerceptUnionBelief((PerceptUnionBelief) belief, mln);
		}
		
		// tracking
		else if (belief instanceof MultiModalBelief) {
			result =  inferMultiModalBelief((MultiModalBelief) belief, mln);
		}
		
		// temporal smoothing
		else if (belief instanceof TemporalUnionBelief) {
			result =  inferTemporalUnionBelief((TemporalUnionBelief) belief, mln);
		}
		
		// something is wrong here
		else {
			throw new MLException("can not perform inference on belief of type: "
					+ belief.getClass().getCanonicalName());
		}
		
		// the belief we used for inference has to be removed again
		removeBelief(belief.id);
		
		return result;
	}
	private Map<Belief, Float> inferMultiModalBelief(MultiModalBelief belief, StringBuilder mln) {
		// TODO Auto-generated method stub
		return null;
	}

	private Map<Belief, Float> inferPerceptBelief(PerceptBelief belief, StringBuilder mln) throws MLException {
		
		runMarkovLogicInference(mln, INFER_PERCEPTUAL_GROUPING + "(" + getInternalName(belief.id) + ",to_infer)\n");
		
		// process the results
		Map<Long, Float> results_from_alchemy = parseResultsFromFile();
		
		Map<Belief, Float> results = new HashMap<Belief, Float>();
		for(Long internal_id : results_from_alchemy.keySet()) {
			Belief b = belief_id_to_belief.get(this.belief_id_to_internal_id.getKeyFromValue(internal_id));
			results.put(b, results_from_alchemy.get(internal_id));
		}
		
		return results;
	}
	private Map<Belief, Float> inferPerceptUnionBelief(PerceptUnionBelief belief, StringBuilder mln) {
		// TODO Auto-generated method stub
		return null;
	}

	private Map<Belief, Float> inferTemporalUnionBelief(TemporalUnionBelief belief, StringBuilder mln) {
		// TODO Auto-generated method stub
		return null;
	}

	public void init(MLPreferences preferences) {
		this.preferences = preferences;
	}

	private Map<Long, Float> parseResultsFromFile() throws MLException {
		Map<Long, Float> results = new TreeMap<Long, Float>();

		FileReader file_reader = null;
		try {
			file_reader = new FileReader(preferences.getPath_to_output());

		} catch (FileNotFoundException e) {
			throw new MLException("could not open results file\n"
					+ e.getStackTrace());
		}

		BufferedReader buffered_reader = new BufferedReader(file_reader);

		String line = null;
		try {
			while (buffered_reader.ready()) {
				line = buffered_reader.readLine();
				if(line.length() > 0) {
					Float probability = getProbability(line);
					Long internal_belief_id = getBeliefID(line);
					results.put(internal_belief_id, probability);
				}
			}
		} catch (IOException e) {
			throw new MLException("error while parsing results\n"
					+ e.getStackTrace());
		}
		return results;
	}

	// /////////////////////////////////////////////////////////////////////////////////////
	//
	// Helper functions
	//
	// /////////////////////////////////////////////////////////////////////////////////////
	
	/**
	 * Used in the parser of the alchemy result files to extract the BeliefID
	 * from a result of the form "Predicate(NewBeliefID,BeliefID) <PROBABILITY>"
	 */
	private Long getBeliefID(String line) {
		System.out.println(line);
		String belief_name = line.split(",")[1].split("\\)")[0];
		assert belief_name.startsWith(PNTR_PREFIX);
		return new Long(belief_name.replace(PNTR_PREFIX, ""));
	}

	private Float getProbability(String line) {
		return new Float(line.split(" ")[1]);
	}

	public void removeBelief(String belief_id) throws MLException {
		if (!beliefs_it_to_mlformula.containsKey(belief_id)) {
			throw new MLException("Belief does not exist here... invalid ID");
		}
		
		predicate_data.removeBelief(belief_id);
		beliefs_it_to_mlformula.remove(belief_id);
		belief_id_to_belief.remove(belief_id);
		
		for (Set<Belief> ids : types_of_beliefs.values()) {
			for (Belief belief : ids) {
				if (belief.id.equals(belief_id)) {
					ids.remove(belief);
					return;
				}
			}
		}
		
		belief_id_to_internal_id.remove(belief_id);
	}

	private void runMarkovLogicInference(StringBuilder mln, String query)
			throws MLException {

		// 1. write the content of MLN to the file-system
		try {
			FileWriter file_writer = new FileWriter(preferences
					.getPath_to_input());
			BufferedWriter buffered_writer = new BufferedWriter(file_writer);
			buffered_writer.write(mln.toString());
			buffered_writer.close();
		} catch (Exception e) {
			throw new MLException("Could not write mln to file");
		}

		// 2. call mln for query
		String[] command = getCommand(query);
		
		String disp = "";
		for(String s : command) {
			disp = disp + s;
		}
		
		System.out.println("Command to exec: " + disp);
		Process infer = null;
		try {
			infer = Runtime.getRuntime().exec(disp);
		} catch (IOException e) {
			throw new MLException("call to alchemy failed\n"
					+ e.getStackTrace());
		}

		int returncode = 0;
		try {
			returncode = infer.waitFor();
		} catch (InterruptedException e) {
			throw new MLException(
					"interrupted while waiting for alchemy to finish\n"
							+ e.getStackTrace());
		}

		if (returncode != 0) {
			BufferedInputStream std_out = new BufferedInputStream(infer.getInputStream());
			BufferedInputStream std_err = new BufferedInputStream(infer.getErrorStream());
			
			int ch;
			StringBuffer sb = new StringBuffer();
			try {
				// read process standard output stream
				while ((ch = std_out.read()) != -1) {
					sb.append((char)ch);
				}
				std_out.close();
				// read process standard error stream
				while ((ch = std_err.read()) != -1) {
					sb.append((char)ch);
				}
				std_err.close();
			} catch (IOException e) {
				// do nothing here, we through an exception anyway
			}
			
			throw new MLException("Non zero return code from Alchemy" + sb.toString());
		}
	}

	// /////////////////////////////////////////////////////////////////////////////////////
	//
	// Functions for JUnit tests
	//
	// /////////////////////////////////////////////////////////////////////////////////////

	public Set<String> testGetFeatureAlternatives(Belief belief, String feature) {
		return predicate_data
				.getValuesForType(feature.toString().toLowerCase());
	}

	public boolean testHasFeature(Belief belief, String feature) {
		return predicate_data.hasBeliefForPredicate(feature.toString(),
				belief.id);
	}

	public void updateBelief(String id, Belief belief) throws MLException {
		removeBelief(id);
		addBelief(belief);
	}
}
