package de.dfki.lt.tr.cast.dialogue;

public class SilentStandbyModeManager
extends AbstractStandbyModeManager {

	@Override
	public ProcessingTask onStandbyOn() {
		return null;
	}

	@Override
	public ProcessingTask onStandbyOff() {
		return null;
	}
	
}
