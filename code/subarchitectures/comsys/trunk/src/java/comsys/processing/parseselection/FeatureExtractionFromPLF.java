package comsys.processing.parseselection;

import java.util.Hashtable;
import java.util.Vector;
import java.util.Enumeration;

import java.util.StringTokenizer;

import comsys.datastructs.comsysEssentials.NonStandardRule;
import comsys.datastructs.comsysEssentials.NonStandardRulesAppliedForLF;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.comsysEssentials.PhonStringLFPair;
import comsys.datastructs.lf.PackedFeature;
import comsys.datastructs.lf.PackedNominal;
import comsys.datastructs.lf.PackingNode;
import comsys.datastructs.lf.PackingNodeTarget;
import comsys.lf.utils.LFUtils;

public class FeatureExtractionFromPLF extends FeatureExtraction {

	public boolean extractSemanticFeatures = true;
	public boolean extractSyntacticFeatures = true;
	public boolean extractAcousticFeatures = true;
	public boolean extractContextualFeatures = true;
	public boolean extractNoParseFeature = true;
	
	public String contextdependentwordsFilename = "";
	public String salientwordsFilename = "";
	
	Hashtable<String,FeatureVector> featureVectors;

	PackedLFs plf;

	public FeatureExtractionFromPLF (PackedLFs plf) {

		featureVectors = new Hashtable<String,FeatureVector>();
		
		this.plf = plf;
		
	//	LFUtils.plfToGraph(plf.packedLF, "THEplf", true);
		// create a new (emtpy) feature vector for each logical form in the PLF
		if (plf.packedLF != null && plf.packedLF.root != null) {
		PackingNode root = LFUtils.plfGetPackingNode(plf.packedLF, plf.packedLF.root);
		for (int i = 0; i < root.lfIds.length ; i++) {
			featureVectors.put(root.lfIds[i], new FeatureVector());
		}
		
		for (int i = 0; i < plf.nonParsablePhonStrings.length ; i++) {
			featureVectors.put("nonparsable-"+i, new FeatureVector());
		}
		}
	}

	public Hashtable<String,FeatureVector> extractFeatures() {
		if (extractSemanticFeatures)
			extractSemanticFeatures();
		if (extractSyntacticFeatures)
			extractSyntacticFeatures();
		if (extractAcousticFeatures)
			extractAcousticFeatures();
		if (extractContextualFeatures)
			extractContextualFeatures(); 
		if (extractNoParseFeature)
			extractNoParseFeature(); 	
		return featureVectors;
	}

	public Hashtable<String,FeatureVector> extractSemanticFeatures() {

		// populate the feature vector with semantic features
		
		if (plf.packedLF != null) {
		
		for (int i = 0; i < plf.packedLF.pNodes.length ; i ++ ) {
			PackingNode pNode = plf.packedLF.pNodes[i];
			for (int j = 0 ; j < pNode.packedNoms.length ; j++) {

				// lexical features
				PackedNominal pNom = pNode.packedNoms[j];
				//	String headSort = getSort(pNom);
				String headProp = pNom.prop.prop;

				for (int z=0; z < pNom.packedSorts.length; z++) {
					String headSort = pNom.packedSorts[z].sort;
					String[] lfIds = pNom.packedSorts[z].lfIds;

					updateLexicalFeaturesPLF(headSort, headProp, lfIds);
					updateLexicalFeatures2PLF(headSort, lfIds);

					for (int y=0; y < pNom.feats.length; y++) {
						PackedFeature pFeat = pNom.feats[y];
						String feature = pFeat.feat;
						String value = pFeat.value;
						String[] lfIds2 = pFeat.lfIds;	
						String[] interLfIds = intersection(lfIds, lfIds2);
						updateNominalFeatureFeaturesPLF(headSort, feature, value, interLfIds);
					}


					// dependency features in internal LF relations
					for (int k=0; pNom.rels != null && k < pNom.rels.length; k++) {

						String mode = pNom.rels[k].mode;
						PackedNominal depNom = LFUtils.plfGetPackedNominal(pNode, pNom.rels[k].dep);

						updateDependencyFeatures2PLF(headSort, headProp, mode, lfIds);
						
						for (int y=0; y < depNom.packedSorts.length; y++) {
							String depSort = depNom.packedSorts[y].sort;
							String[] lfIds2 = depNom.packedSorts[y].lfIds;	
							String[] interLfIds = intersection(lfIds, lfIds2);
							updateDependencyFeaturesPLF(headSort, depSort, mode, interLfIds);		
						}
						
						for (int l=0; depNom.rels != null && l < depNom.rels.length; l++) {
							String mode2 = depNom.rels[l].mode;
							updateBigramDependencyFeaturesPLF(mode, mode2, lfIds);
							updateBigramDependency2FeaturesPLF(headSort, mode, mode2, lfIds);
						}
						
						for (int l=0 ; depNom.pEdges != null && l < depNom.pEdges.length ; l++) {
							String mode2 = depNom.pEdges[l].mode;
							for (int m=0; m < depNom.pEdges[l].targets.length ; m++) {
								PackingNodeTarget target = depNom.pEdges[l].targets[m];
								String[] interLfIds = intersection(lfIds, target.lfIds);
								updateBigramDependencyFeaturesPLF(mode, mode2, interLfIds);
								updateBigramDependency2FeaturesPLF(headSort, mode, mode2, interLfIds);
							}
						}
					}

					// dependency features in packed edges
					for (int k=0 ; pNom.pEdges != null && k < pNom.pEdges.length ; k++) {

						String mode = pNom.pEdges[k].mode;

						for (int l=0; l < pNom.pEdges[k].targets.length ; l++) {
							PackingNodeTarget target = pNom.pEdges[k].targets[l];
							PackingNode pDepNode = LFUtils.plfGetPackingNode(plf.packedLF, target.pnId);
							PackedNominal depRootNom = LFUtils.plfGetPackedNominal(pDepNode, pDepNode.root);

							String[] interLfIds = intersection(lfIds, target.lfIds);

							updateDependencyFeatures2PLF(headSort, headProp, mode, interLfIds);

							for (int y=0; y < depRootNom.packedSorts.length; y++) {
								String depSort = depRootNom.packedSorts[y].sort;
								String[] lfIds2 = depRootNom.packedSorts[y].lfIds;	
								String[] interLfIds2 = intersection(interLfIds, lfIds2);
								updateDependencyFeaturesPLF(headSort, depSort, mode, interLfIds2);
							}
							
							for (int m=0; depRootNom.rels != null && m < depRootNom.rels.length; m++) {
								String mode2 = depRootNom.rels[m].mode;
								updateBigramDependencyFeaturesPLF(mode, mode2, interLfIds);
								updateBigramDependency2FeaturesPLF(headSort, mode, mode2, interLfIds);
							}
							
							for (int m=0 ; depRootNom.pEdges != null && m < depRootNom.pEdges.length ; m++) {
								String mode2 = depRootNom.pEdges[m].mode;
								for (int n=0; n < depRootNom.pEdges[m].targets.length ; n++) {
									PackingNodeTarget target2 = depRootNom.pEdges[m].targets[n];
									String[] interLfIds2 = intersection(interLfIds, target2.lfIds);
									updateBigramDependencyFeaturesPLF(mode, mode2, interLfIds2);
									updateBigramDependency2FeaturesPLF(headSort, mode, mode2, interLfIds2);
								}
							}
						}
					}
				}
			}
		}

		for (int i = 0; i < plf.packedLF.pNodes.length ; i ++ ) {
			PackingNode pNode = plf.packedLF.pNodes[i];
			String[] lfIds = pNode.lfIds;
			updateNbrNominalsFeaturesPLF (pNode.packedNoms.length, lfIds);

			for (int j = 0 ; j < pNode.packedNoms.length ; j++) {
				PackedNominal pNom = pNode.packedNoms[j];
				if (pNom.prop.prop.equals("")) {
					updateNbrEmptyNominalsFeaturesPLF (1, lfIds);
				}
			}
		}

		}
		
		return featureVectors;
	}
	
	public Hashtable<String,FeatureVector> extractSyntacticFeatures() {
		
		for (int i= 0; i < plf.nonStandardRulesForLF.length; i++) {
			
			NonStandardRulesAppliedForLF nonStandardRulesForLF = plf.nonStandardRulesForLF[i];
			String lfId = nonStandardRulesForLF.logicalFormId;
			FeatureVector fv = featureVectors.get(lfId);
			for (int j=0; j < nonStandardRulesForLF.nonStandardRules.length; j++) {
				NonStandardRule rule = nonStandardRulesForLF.nonStandardRules[j];
				String rulename = rule.rulename;
				int number = (int) rule.numberOfApplications;
				String featName = getNonStandardRuleFeatName(rulename);
				fv.addFeature(featName, number);
			}
		}
		
		
		return featureVectors;
	}
	
	public Hashtable<String,FeatureVector> extractAcousticFeatures() {
	
		String lfIdWithHighestConfValue = "";
		double highestConfValue = 0.0;
		double secondHighestConfValue = 0.0;
		
		for (int i= 0; i < plf.phonStringLFPairs.length; i++) {		
			PhonStringLFPair pair = plf.phonStringLFPairs[i];
			String lfId = pair.logicalFormId;

			FeatureVector fv = featureVectors.get(lfId);
			String featName1 = getConfidenceValueFeatName();
			fv.addFeature(featName1, pair.phonStr.confidenceValue);
			String featName2 = getNLConfidenceValueFeatName();
			fv.addFeature(featName2, pair.phonStr.NLconfidenceValue);
			String featName3 = getRankFeatName();
			fv.addFeature(featName3, pair.phonStr.rank);	
			if(pair.phonStr.confidenceValue > 50.0) {
				String featName4 = getConfidenceValueHigherThan50FeatName();
				fv.addFeature(featName4, 1.0);
			}
			if (pair.phonStr.confidenceValue > highestConfValue) {
				highestConfValue = pair.phonStr.confidenceValue;
				lfIdWithHighestConfValue = lfId;
			}		
			else if (pair.phonStr.confidenceValue > secondHighestConfValue) {
				secondHighestConfValue = pair.phonStr.confidenceValue;
			}
		}
		for (int i = 0; i < plf.nonParsablePhonStrings.length; i++) {
			PhonString phon = plf.nonParsablePhonStrings[i];
			FeatureVector fv = featureVectors.get("nonparsable-"+i);
			String featName1 = getConfidenceValueFeatName();
			fv.addFeature(featName1, phon.confidenceValue);
			String featName2 = getNLConfidenceValueFeatName();
			fv.addFeature(featName2, phon.NLconfidenceValue);
			String featName3 = getRankFeatName();
			fv.addFeature(featName3, phon.rank);
			if(phon.confidenceValue > 50.0) {
				String featName4 = getConfidenceValueHigherThan50FeatName();
				fv.addFeature(featName4, 1.0);
			}
			if (phon.confidenceValue > highestConfValue) {
				highestConfValue = phon.confidenceValue;
				lfIdWithHighestConfValue = "nonparsable-"+i;
			}
			else if (phon.confidenceValue > secondHighestConfValue) {
				secondHighestConfValue = phon.confidenceValue;
			}
		}
		
		if (highestConfValue > 0.0 && secondHighestConfValue > 0 &&
				highestConfValue > (secondHighestConfValue + 5)) {
			FeatureVector fv = featureVectors.get(lfIdWithHighestConfValue);
			String featName5 = getConfidenceValueSignificantlyHigherFeatName();
			fv.addFeature(featName5, 1.0);
		}
		
		return featureVectors;
	}
	
	public Hashtable<String,FeatureVector> extractContextualFeatures() {
		
		Vector<String> contextuallyActivatedWords = new Vector<String>();
		Vector<String> nonContextuallyActivatedWords = new Vector<String>();
		
		Vector<String> salientWords = extractWords(salientwordsFilename);
		Vector<String> contextdependentWords = extractWords(contextdependentwordsFilename);
		
		for (Enumeration<String> e = contextdependentWords.elements(); e.hasMoreElements();) {
			String word = e.nextElement();
	//		log("Contextually activated word added: " + entity.getConcept());
			if (salientWords.contains(word)) {
				contextuallyActivatedWords.add(word);
			}
			else {
				nonContextuallyActivatedWords.add(word);
			}
		}
		
		for (int i= 0; i < plf.phonStringLFPairs.length; i++) {
			PhonStringLFPair pair = plf.phonStringLFPairs[i];
			String lfId = pair.logicalFormId;
			FeatureVector fv = featureVectors.get(lfId);
			StringTokenizer tokenizer = new StringTokenizer(pair.phonStr.wordSequence);
			double nonContextual = 0.0;
			while (tokenizer.hasMoreTokens()) {
				String word = tokenizer.nextToken();
				if (contextuallyActivatedWords.contains(word)) {
					String featname = getWordActivationFeatName(word);
					fv.addFeature(featname, 1.0);
		//			log("contextual word " + word + " activated");
				}
				else if (nonContextuallyActivatedWords.contains(word)) {
					nonContextual++;
				}
			}
			if (nonContextual > 0.0) {
				String featname = getWordActivationNonContextualFeatName();
				fv.addFeature(featname, nonContextual);
		//		log("number of non-contextual words for " + pair.phonString.wordSequence + ": " + nonContextual);
			}			
		}

		for (int i = 0; i < plf.nonParsablePhonStrings.length; i++) {
			PhonString phon = plf.nonParsablePhonStrings[i];
			FeatureVector fv = featureVectors.get("nonparsable-"+i);
			StringTokenizer tokenizer = new StringTokenizer(phon.wordSequence);
			double nonContextual = 0.0;
			while (tokenizer.hasMoreTokens()) {
				String word = tokenizer.nextToken();
				if (contextuallyActivatedWords.contains(word)) {
					String featname = getWordActivationFeatName(word);
					fv.addFeature(featname, 1.0);
				}
				else if (nonContextuallyActivatedWords.contains(word)) {
					nonContextual++;
				}
			}
			if (nonContextual > 0.0) {
				String featname = getWordActivationNonContextualFeatName();
				fv.addFeature(featname, nonContextual);
		//		log("number of non-contextual words for " + phon.wordSequence + ": " + nonContextual);
			}		
		}
		return featureVectors;
	}

	
	public Hashtable<String,FeatureVector> extractNoParseFeature() {
		for (int i = 0; i < plf.nonParsablePhonStrings.length; i++) {
			PhonString phon = plf.nonParsablePhonStrings[i];
			FeatureVector fv = featureVectors.get("nonparsable-"+i);
			String featname = getNoParseFeatName();
			fv.addFeature(featname, 1.0);
		}
		return featureVectors;
	}

	private void updateDependencyFeaturesPLF(String headSort, String depSort, String mode, String[] lfIds){
		if (!headSort.equals("") && !depSort.equals("") && !mode.equals("")) {
			String depFeatureName = 
				getDependencyFeatName(headSort, depSort, mode);
			//		log("dependency feature extracted: " + depFeatureName);
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				featVec.incrementFeatureValue(depFeatureName);
			}
		}
	}

	private void updateDependencyFeatures2PLF(String headSort, String headProp, String mode, String[] lfIds){
		if (!headSort.equals("") && !headProp.equals("") && !mode.equals("")) {
			String depFeature2Name = 
				getDependency2FeatName(headSort, headProp, mode);
			//		log("dependency feature extracted: " + depFeatureName);
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				featVec.incrementFeatureValue(depFeature2Name);
			}
		}
	}
	
	private void updateBigramDependencyFeaturesPLF(String mode1, String mode2, String[] lfIds){
		if (!mode1.equals("") && !mode2.equals("")) {
			String bigramFeatName = 
				getBigramDependencyFeatName(mode1, mode2);
			//		log("dependency feature extracted: " + depFeatureName);
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				featVec.incrementFeatureValue(bigramFeatName);
			}
		}
	}
	
	private void updateBigramDependency2FeaturesPLF(String headSort, String mode1, String mode2, String[] lfIds){
		if (!headSort.equals("") && !mode1.equals("") && !mode2.equals("")) {
			String bigram2FeatName = 
				getBigramDependency2FeatName(headSort, mode1, mode2);
			//		log("dependency feature extracted: " + depFeatureName);
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				featVec.incrementFeatureValue(bigram2FeatName);
			}
		}
	}


	private void updateLexicalFeaturesPLF(String sort, String prop, String[] lfIds) {			
		if (!sort.equals("") && !prop.equals("")) {
			String lexicaFeatureName = getLexicalFeatName(sort, prop);
			//		log("lexical feature extracted: " + lexicaFeatureName);
			for (int k = 0; k < lfIds.length ; k++) {
				FeatureVector featVec = featureVectors.get(lfIds[k]);
				featVec.incrementFeatureValue(lexicaFeatureName);
			}
		}
	}

	private void updateLexicalFeatures2PLF(String sort, String[] lfIds) {			
		if (!sort.equals("")) {
			String lexical2FeatureName = getLexical2FeatName(sort);
			//		log("lexical feature extracted: " + lexicaFeatureName);
			for (int k = 0; k < lfIds.length ; k++) {
				FeatureVector featVec = featureVectors.get(lfIds[k]);
				featVec.incrementFeatureValue(lexical2FeatureName);
			}
		}
	}

	private void updateNominalFeatureFeaturesPLF(String sort, String feat, String value, String[] lfIds){
		if (!sort.equals("") && !feat.equals("") && !value.equals("")) {
			String depFeatureName = 
				getNominalFeatureFeatName(sort, feat, value);
			//		log("dependency feature extracted: " + depFeatureName);
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				featVec.incrementFeatureValue(depFeatureName);
			}
		}
	}

	private void updateNbrNominalsFeaturesPLF(int increment, String[] lfIds) {
		if (increment > 0) {
			String nbrNominalsFeatureName = getNbrNominalsFeatName();
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				double newValue = featVec.getFeatureValue(nbrNominalsFeatureName) + increment;
				featVec.setFeatureValue(nbrNominalsFeatureName, newValue);
			}
		}
	}

	private void updateNbrEmptyNominalsFeaturesPLF(int increment, String[] lfIds) {
		if (increment > 0) {
			String nbrEmptyNominalsFeatureName = getNbrEmptyNominalsFeatName();
			for (int m = 0; m < lfIds.length ; m++) {
				FeatureVector featVec = featureVectors.get(lfIds[m]);
				double newValue = featVec.getFeatureValue(nbrEmptyNominalsFeatureName) + increment;
				featVec.setFeatureValue(nbrEmptyNominalsFeatureName, newValue);
			}
		}
	}

}
