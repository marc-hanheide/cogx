package de.dfki.lt.tr.dialogue.ref;

import cast.cdl.WorkingMemoryAddress;

public interface ReferenceResolver {

	public static final String SORT_DISCOURSE = "discourse";
	public static final String SORT_OBJECT = "object";
	public static final String SORT_PLACE = "place";
	public static final String SORT_INDETERMINATE = "indeterminate";

	public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin);

}
