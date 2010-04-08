
package beliefmodels.builders;

import beliefmodels.autogen.featurecontent.*;

public class FeatureValueBuilder {

	
	public static StringValue createNewStringValue(String val) {
		return new StringValue(val);
	}
	
	public static BooleanValue createNewBooleanValue(boolean val) {
		return new BooleanValue(val);
	}
	
	public static IntegerValue createNewIntegerValue(int val) {
		return new IntegerValue(val);
	}
	
	public static UnknownValue createNewUnknownValue() {
		return new UnknownValue();
	}
}
