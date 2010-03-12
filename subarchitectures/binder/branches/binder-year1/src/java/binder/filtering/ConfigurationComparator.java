package binder.filtering;

import java.util.Comparator;

import binder.autogen.core.UnionConfiguration;

public class ConfigurationComparator implements Comparator<UnionConfiguration> {

	public int compare(UnionConfiguration o1, UnionConfiguration o2) {
		if ((o1.configProb - o2.configProb) < 0.0f ) {
			return -1;
		}
		else if ((o1.configProb - o2.configProb) > 0.0f ) {
			return 1;
		}
		return 0;
	}




}
