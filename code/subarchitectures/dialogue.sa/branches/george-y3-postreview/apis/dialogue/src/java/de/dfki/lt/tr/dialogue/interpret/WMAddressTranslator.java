package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;

public interface WMAddressTranslator {

	public WorkingMemoryAddress translate(WorkingMemoryAddress addr);

}
