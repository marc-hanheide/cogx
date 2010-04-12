
package beliefmodels.builders;

import beliefmodels.autogen.featurecontent.*;
import beliefmodels.arch.BeliefException;


public class FeatureValueBuilder {

	
	public static StringValue createNewStringValue(String val) throws BeliefException {
		if (val != null) {
			if (!val.equals("")) { 
				return new StringValue(val);
			} else { 
				throw new BeliefException("StringValue cannot be set to an empty value");
			}
		} else { 
			throw new BeliefException("StringValue cannot be set to a null value");
		}
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
