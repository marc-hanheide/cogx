package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import java.util.HashMap;
import java.util.Map;

public class GeneratedWMAddressTranslator
implements WMAddressTranslator {

	private final Map<WorkingMemoryAddress, WorkingMemoryAddress> cache;
	private final IdentifierGenerator<WorkingMemoryAddress> idGen;
	
	public GeneratedWMAddressTranslator(IdentifierGenerator<WorkingMemoryAddress> idGen) {
		cache = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
		this.idGen = idGen;
	}

	@Override
	public WorkingMemoryAddress translate(WorkingMemoryAddress addr) {
		WorkingMemoryAddress newAddr = cache.get(addr);
		if (newAddr == null) {
			newAddr = idGen.newIdentifier();
			cache.put(addr, newAddr);
		}
		return newAddr;
	}


}
