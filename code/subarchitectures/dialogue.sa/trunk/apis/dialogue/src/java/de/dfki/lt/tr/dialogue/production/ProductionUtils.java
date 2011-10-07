package de.dfki.lt.tr.dialogue.production;

import cast.cdl.WorkingMemoryAddress;

public class ProductionUtils {

	public static String referenceGenerationRequestToString(ReferenceGenerationRequest request) {
		if (request == null) {
			return "NULL";
		}
		String s = "(GRE request\n";
		s += "  about = \"" + wmaToString(request.obj) + "\"\n";
		s += "  shortNP = " + Boolean.toString(request.shortNP) + "\n";
		s += "  spatialRelation = " + Boolean.toString(request.spatialRelation) + "\n";
		s += "  disabledProperty = \"" + request.disabledProperty + "\"\n";
		s += ")";
		return s;
	}

	public static String referenceGenerationResultToString(ReferenceGenerationResult result) {
		if (result == null) {
			return "NULL";
		}
		String s = "(GRE result (in response to " + wmaToString(result.requestAddress) + ")\n";
		s += "  refEx = \"" + result.refEx + "\"\n";
		s += ")";
		return s;
	}

	public static String wmaToString(WorkingMemoryAddress wma) {
		return "[" + wma.id + "," + wma.subarchitecture + "]";
	}

}
