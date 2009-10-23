package comsys.processing.parseselection;

import comsys.datastructs.lf.Feature;
import comsys.datastructs.lf.LFNominal;
import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;


public class FeatureExtractionFromLF extends FeatureExtraction {

	LogicalForm lf;
	
	public FeatureExtractionFromLF(LogicalForm lf) {
		this.lf = lf;
	}
	

	public FeatureVector extractSemanticFeatures() {

		FeatureVector features = new FeatureVector();
		
		for (int i=0; i < lf.noms.length ; i++) {
			LFNominal nom = lf.noms[i];
			String headSort = nom.sort;
			String headProp = nom.prop.prop;
		
			if (!headSort.equals("") && !headProp.equals("")) {
				String lexicaFeatureName = getLexicalFeatName(headSort, headProp);
			//	log("lexical feature extracted: " + lexicaFeatureName);
				features.incrementFeatureValue(lexicaFeatureName);
			}
			
			if (!headSort.equals("")) {
				String lexicaFeature2Name = getLexical2FeatName(headSort);
			//	log("lexical feature extracted: " + lexicaFeatureName);
				features.incrementFeatureValue(lexicaFeature2Name);
			}
			
			for (int j = 0; j < nom.feats.length; j++) {
				Feature feature = nom.feats[j];
				String feat = feature.feat;
				String value = feature.value;
				if (!headSort.equals("") && !feat.equals("") && !value.equals("")) {
					String featureFeatureName = getNominalFeatureFeatName(headSort, feat, value);
				//	log("nominal feature feature extracted: " + lexicaFeatureName);
					features.incrementFeatureValue(featureFeatureName);
				}
			}
				
			for (int j = 0; j < nom.rels.length ; j++) {
				LFNominal depNom = LFUtils.lfGetNominal(lf, nom.rels[j].dep);
				
				if (depNom != null) {
				String depSort = depNom.sort;
				String mode = nom.rels[j].mode;
					
				if (!headSort.equals("") && !depSort.equals("") && !mode.equals("")) {
					String depFeatureName = getDependencyFeatName(headSort, depSort, mode);
			//		log("dependency feature extracted: " + depFeatureName);
					features.incrementFeatureValue(depFeatureName);
				}
				
				if (!headSort.equals("") && !headProp.equals("") && !mode.equals("")) {
					String dep2FeatureName = getDependency2FeatName(headSort, headProp, mode);
			//		log("dependency feature extracted: " + depFeatureName);
					features.incrementFeatureValue(dep2FeatureName);
				}
				
				for (int k = 0; k < depNom.rels.length ; k++) {
					String mode2 = depNom.rels[k].mode;
					String bigramFeatName = getBigramDependencyFeatName(mode, mode2);
					//		log("dependency feature extracted: " + depFeatureName);
					features.incrementFeatureValue(bigramFeatName);	
					
					String bigram2FeatName = getBigramDependency2FeatName(headSort, mode, mode2);
					//		log("dependency feature extracted: " + depFeatureName);
					features.incrementFeatureValue(bigram2FeatName);	
				}
			}
			}
		}
		String nbrNominals = getNbrNominalsFeatName();
		features.setFeatureValue(nbrNominals, lf.noms.length);
		
		String nbrEmptyNominals = getNbrEmptyNominalsFeatName();
		for (int i=0; i < lf.noms.length ; i++) {
			LFNominal nom = lf.noms[i];
			if (nom.prop.prop.equals("")) {
				double newValue = features.getFeatureValue(nbrEmptyNominals)  + 1;
				features.setFeatureValue(nbrEmptyNominals, newValue);
			}
		}
		if (!features.hasFeature(nbrEmptyNominals)) {
			features.setFeatureValue(nbrEmptyNominals, 0);
		}

		return features;
	}
		
}
