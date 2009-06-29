

module binder {

module autogen {


class FeatureDistribution {
	string featureLabel;
};

sequence<FeatureDistribution> MultiFeatureDistribution;

interface FeatureValueDistribution {
};


class DiscreteFeatureValueDistribution implements FeatureValueDistribution {
};


interface FeatureValue {
};

class StringValue {
	string val;
};

class DiscreteFeatureValueWithProb {
	FeatureValue featValue;
	float prob;
};

class PerceivedEntity {
	float probExists;
	MultiFeatureDistribution features;
};


class Proxy extends PerceivedEntity {
	string proxyId;
	string subarchId;
};

sequence<Proxy> ProxySeq;

class Union extends PerceivedEntity {
	string unionId;
	ProxySeq includedProxies;
};

};
};
