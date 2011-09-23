package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.dialogue.parseselection.SimpleParseSelector;
import java.util.Map;

public class SimpleParseSelection
extends AbstractParseSelectionComponent<SimpleParseSelector> {

	public SimpleParseSelection() {
		super(new SimpleParseSelector());
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);
		getSelector().setLogger(getLogger(".selector"));
	}

}
