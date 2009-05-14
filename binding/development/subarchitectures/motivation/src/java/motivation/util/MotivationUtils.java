package motivation.util;

import motivation.idl.MotiveType;

public class MotivationUtils {

	public static String toString(MotiveType _type) {
		switch (_type.value()) {
			case MotiveType._CLARIFICATION :	
				return "CLARIFICATION";
			case MotiveType._FACTUAL_QUESTION :	
				return "FACTUAL_QUESTION";
			case MotiveType._PHYSICAL_ACTION :	
				return "PHYSICAL_ACTION";
			case MotiveType._POLAR_QUESTION :	
				return "POLAR_QUESTION";
			default :
				throw new RuntimeException("Not handled enum value: " + _type);
		}
	}
	
}
