package binder.builders;

import binder.autogen.featurecontent.BooleanValue;
import binder.autogen.featurecontent.IntegerValue;
import binder.autogen.featurecontent.UnknownValue;
import binder.autogen.featurecontent.StringValue;

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
