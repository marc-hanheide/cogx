package de.dfki.lt.tr.dialogue.intentions;

public abstract class AbstractRichIntention implements RichIntention {

	public AbstractRichIntention() {
	}

	@Override
	public CASTEffect getOnAcceptEffect() {
		return CASTEffect.NoOpEffect.INSTANCE;
	}
	
	@Override
	public CASTEffect getOnSuccessEffect() {
		return CASTEffect.NoOpEffect.INSTANCE;
	}

}
