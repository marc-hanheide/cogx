package binder.utils;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.combined.CombinedProbabilityDistribution;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.specialentities.RelationUnion;


/**
 * 
 * @author plison
 *
 */
public class ConfigurationFilter {


	public static boolean LOGGING = false;

	
	public static Vector<UnionConfiguration> getNBestUnionConfigurations
	(Vector<UnionConfiguration> configs, int nb_nbests) {

		double threshold = 0.0f;
		Vector<UnionConfiguration> nbestConfigs = new Vector<UnionConfiguration>();
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();
			if (nbestConfigs.size() < nb_nbests) {

				nbestConfigs.add(config);

				if (config.configProb < threshold) {
					threshold = config.configProb;
				}
			}

			else {
				if (config.configProb > threshold) {
					UnionConfiguration worstinNBests = getWorstUnionConfiguration(nbestConfigs);
					nbestConfigs.remove(worstinNBests);
					nbestConfigs.add(config);
					UnionConfiguration secondworst = getWorstUnionConfiguration(nbestConfigs);
					threshold = secondworst.configProb;
				}
			}
		}

		return nbestConfigs;
	}

	
	public static UnionConfiguration getBestUnionConfiguration(Vector<UnionConfiguration> configs) {

		double maxAverage = -1.0f;
		UnionConfiguration bestConfig = null;

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

			if (config.configProb > maxAverage) {
				maxAverage = config.configProb;
				bestConfig = config;
			}
		}

		return bestConfig;
	}



	public static UnionConfiguration getWorstUnionConfiguration(Vector<UnionConfiguration> configs) {

		double minAverage = 99999.0f;
		UnionConfiguration worstConfig = null;

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

				if (config.configProb < minAverage) {
					minAverage = config.configProb;
					worstConfig = config;
				}
		}

		return worstConfig;
	}


	public static UnionConfiguration getBestUnionConfiguration
		(AlternativeUnionConfigurations configs) {
		Vector<UnionConfiguration> unionconfigsV = new Vector<UnionConfiguration>();
		for (int i = 0 ; i < configs.alterconfigs.length ; i++) {
			unionconfigsV.add(configs.alterconfigs[i]);
		}
		return getBestUnionConfiguration (unionconfigsV);
	}



	public static void computeConfidenceScoresForUnionConfigurations
	(Vector<UnionConfiguration> configs) {

		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			UnionConfiguration config = e.nextElement();

			double multiplication = 1.0f;
			for (int i = 0; i < config.includedUnions.length ; i++) {	
				Union union = config.includedUnions[i];
				float max = MaximumSearch.getMaximum(union);
				multiplication = multiplication * max;
				
			} 
			
			config.configProb = multiplication;
			log("configProb: " + config.configProb);
		}
		
	}
	

	
	public static void log(String s) {
		if (LOGGING)
		System.out.println("[GradientDescent] " + s);
	}

}
