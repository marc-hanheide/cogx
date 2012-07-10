package de.dfki.lt.tr.cast.dialogue.planverb;

import java.util.Map;
import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import de.dfki.tarot.cogx.CogXJavaHelpers;
import de.dfki.tarot.cogx.FeatureReplacer;
import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.nlp.lf.BasicState;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;

public class PEVUtils {

	/**
	 * This method performs the lexical substitution pre-processing step.
	 * 
	 * @param blf
	 * @return a BasicLogicalForm that doesn't contain any of the specified out-of-vocabulary words, but temporary replacement words
	 * @throws BuildException
	 * @throws ParseException
	 */
	public static BasicLogicalForm preProcessLexiconSubstitution(BasicLogicalForm blf, Map<String,String> preLexicalSub) throws BuildException, ParseException {
		boolean changed = false;
		String lfString = blf.toString();
		
		if (lfString.contains("be1") || lfString.contains("be2")) {
			// be1 ^ <Cop-Restr>(=var_?block_?step:thing ^
			// be2 ^ <Cop-Restr>(room_?block_?step:e-place ^
			String matchPattern = "(be)([12]+)(.*?)( \\^ <Cop-Restr>)(\\([0-9a-zA-Z_\\-\\:]+)( \\^)";
			String replacePattern = "$1 $3 ^ <Subject>$5) $4$5$6";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
		}

		for (String badWord : preLexicalSub.keySet()) {
			if (lfString.contains(badWord)) {
				lfString = lfString.replace(badWord, preLexicalSub.get(badWord));
				changed = true;
			}
		}
		
		if (!changed) return blf;
		else return BasicLogicalForm.checkedFromString(lfString);
	}
	
	/**
	 * This method performs the lexical substitution post-processing step.
	 * 
	 * @param lfString
	 * @return a String in which temporary replacement words are again substituted with the original out-of-vocabulary words
	 */
	public static String postProcessLexiconSubstitution(String lfString, Map<String,String> postLexicalSub) {

		for (String tmpWord : postLexicalSub.keySet()) {
			if (lfString.contains(tmpWord)) {
				lfString = lfString.replace(tmpWord, postLexicalSub.get(tmpWord));
			}
		}
		
		return lfString;
	}
	
	/**
	 * This method expands several proto features to correct grammar features:
	 * - <ExecutionStatus>SUCCEEDED => <Mood>ind ^ <Tense>past
	 * // - <ExecutionStatus>PENDING => <Mood>ind ^ <Tense>fut
	 * - Subject-Actor agreement
	 * 
	 * @param protoLF
	 * @return a finalized logical form
	 */
    public static BasicLogicalForm finalizeProtoLF(BasicLogicalForm protoLF) {

    	// execution status = success yields past tense report
    	FeatureReplacer pastTenseReplacer = new FeatureReplacer() {
    		@Override
    		public BasicState.Builder doWork(BasicState.Builder builder) {
    			return builder.addFeature("Mood", "ind").addFeature("Tense", "past");
    		}
    	};
    	protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "SUCCEEDED", pastTenseReplacer);
    	protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "PENDING", pastTenseReplacer);

    	FeatureReplacer failedPastTenseReplacer = new FeatureReplacer() {
    		// TODO add <Modifier>(w1:modal ^ could)
    		@Override
    		public BasicState.Builder doWork(BasicState.Builder builder) {
    			return builder.addFeature("Mood", "ind").addFeature("Polarity", "neg").addFeature("Tense", "past"); 
    		}
    	};        
    	protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "FAILED", failedPastTenseReplacer);
    	protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "UNSUCCESSFUL", failedPastTenseReplacer);

    	return protoLF;
    }

	/**
	 * This method performs the WMA substitution pre-processing step.
	 * 
	 * @param blf
	 * @return a BasicLogicalForm that doesn't contain a malformed WMA
	 * @throws BuildException
	 * @throws ParseException
	 */
	public static BasicLogicalForm preProcessWMAs(BasicLogicalForm blf) throws BuildException, ParseException {
		boolean changed = false;
		String lfString = blf.toString();
		
		
		if (lfString.contains("place_")) {
			String matchPattern = "(place_)(_?[0-9a-zA-Z]+)(_)(_?[0-9a-zA-Z]+)(\")";
			String replacePattern = "$2:$4@PLANNERPLACE$5";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
		}
		
		// 1_2_1,room_0__a1
		if (lfString.contains("room_")) {
			String matchPattern = "(room_)(_?[0-9a-zA-Z]+)(_)(_?[0-9a-zA-Z]+)(\")";
			String replacePattern = "$2:$4@COMAROOM$5";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
		}
		
		if (lfString.contains("room")) {
			String matchPattern = "(room)([0-9a-zA-Z]+)(\")";
			String replacePattern = "$1$2@HYPOROOM$3";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
		}
		if (!changed) return blf;
		else return BasicLogicalForm.checkedFromString(lfString);
	}   
    
	/**
	 * Shallow aggregation of surface text
	 * string-based.
	 * 
	 * @param text as single String
	 * @return aggregated text as single String
	 */
	public static String aggregateStrings(String text) {
		String[] sentences = text.split("\n");
		for (int i=0; i < sentences.length -1 ; i++) {
			Pattern pattern = Pattern.compile("(in the room )([a-z]+)( at the place )([a-z]+)");
			Matcher matcher = pattern.matcher(sentences[i]);
			while (matcher.find()) {
				sentences[i+1] = sentences[i+1].replaceAll(matcher.group(), "there");
				sentences[i+1] = "and " + sentences[i+1];
			}
		}
		
		String output = "";
		for (int i=0; i < sentences.length -1; i++) {
			output+=sentences[i];
			if (!sentences[i+1].startsWith("because") && !sentences[i+1].startsWith("and") && !sentences[i+1].startsWith("but")) output += ".\n";
			else output += " ";
		}
		
		return output + sentences[sentences.length-1] + ".";
	}
	
	public static void main(String[] args) {
		String testLF = "@{step_6_1:cognition}(assume ^ <ExecutionStatus>SUCCEEDED ^ <Actor>(\"1_6_1,0:C@spatial.sa\":castreferent ^ \"1_6_1,0:C@spatial.sa\") ^ <Event>(event_6_1:event ^ and ^ <First>(be_6_1_0:ascription ^ be1 ^ <Cop-Restr>(magazine_6_1:thing ^ magazine ^ <Delimitation>existential ^ <Num>sg ^ <Quantification>specific) ^ <Cop-Scope>(in_6_1:m-location ^ in ^ <Anchor>(\"1_6_1,0:81@coma\":castreferent ^ \"1_6_1,0:81@coma\"))) ^ <Next>(be_6_1_1:ascription ^ be2 ^ <Cop-Scope>(meetingroom_6_1:category ^ meetingroom ^ <Delimitation>existential ^ <Num>sg ^ <Quantification>specific) ^ <Cop-Restr>(room_6_1:e-place ^ room ^ <Num>sg ^ <Delimitation>unique ^ <Proximity>proximal ^ <Quantification>specific))) ^ <Subject>(\"1_6_1,0:C@spatial.sa\":castreferent))";
		BasicLogicalForm blf;
		try {
			blf = BasicLogicalForm.checkedFromString(testLF);
			System.out.println(preProcessLexiconSubstitution(blf, new HashMap<String, String>()).toString().replaceAll("\\^", "\\^ \n"));

		} catch (BuildException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

	
}
