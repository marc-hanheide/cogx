package binder.utils;

import java.util.Collection;
import java.util.HashMap;
 
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;

public class MLNGenerator {

	public static String markovlogicDir = "subarchitectures/binder/markovlogic/";

	
	public static void writeMLNFile(PerceptBelief b, Collection<PerceptUnionBelief> existingUnions, 
			HashMap<String,String> unionsMapping, String MLNFileToWrite) {
	
		String constantsSection = constructConstantsSection(existingUnions, unionsMapping);
		String predicatesSection = getPredicatesSection();
		String formulaeForExistingUnions = constructFormulaeForExistingUnions(existingUnions);
		String formulaeForPercept = constructFormulaeForPercept(b);
		String featvaluesconstraintsSection = getFeatValueConstraintsSection();
		String correlationSection = getCorrelationSection();
		String outcomeSection = extractFinalOutcomeSection(existingUnions, unionsMapping);
		
		FileUtils.writeFile(MLNFileToWrite, constantsSection + predicatesSection + formulaeForExistingUnions + 
				formulaeForPercept + featvaluesconstraintsSection + correlationSection + outcomeSection);	
	}
	
	
	
	
	
	private static String extractFinalOutcomeSection(Collection<PerceptUnionBelief> existingUnions, HashMap<String,String> unionsMapping) {
		return FileUtils.readfile(markovlogicDir + "grouping/outcome.mln");
	}
	
	private static String getPredicatesSection() {
		return FileUtils.readfile(markovlogicDir + "grouping/predicates.mln");
	}
	
	private static String getCorrelationSection () {
		return FileUtils.readfile(markovlogicDir + "grouping/correlations.mln");
	}
	
	private static String getFeatValueConstraintsSection () {
		
		return FileUtils.readfile(markovlogicDir + "grouping/featvalueconstraints.mln");
	}
	
	private static String constructFormulaeForExistingUnions (Collection<PerceptUnionBelief> existingUnions) {
		
		return FileUtils.readfile(markovlogicDir + "grouping/existingunions.mln");
	}
	
	private static String constructFormulaeForPercept (PerceptBelief b) {
		
		return FileUtils.readfile(markovlogicDir + "grouping/newpercept.mln");
	}
	
	private static String constructConstantsSection (Collection<PerceptUnionBelief> existingUnions, HashMap<String,String> unionsMapping) {
	
		return FileUtils.readfile(markovlogicDir + "grouping/constantssection.mln");
	}
	
	
	public static String getMarkovLogicConstantFromID (String id) {
		return "U" + id.replace(":", "_");
	}
	
	public static String getIDFromMarkovLogicSontant (String mlconstant) {
		return mlconstant.substring(1).replace("_", ":");
	}
}
