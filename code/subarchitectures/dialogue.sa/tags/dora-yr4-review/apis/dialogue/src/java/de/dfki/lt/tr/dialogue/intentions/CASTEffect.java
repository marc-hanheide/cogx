package de.dfki.lt.tr.dialogue.intentions;

import cast.architecture.ManagedComponent;

public interface CASTEffect {

	public void makeItSo(ManagedComponent component);

	public static final class NoOpEffect implements CASTEffect {

		public static NoOpEffect INSTANCE = new NoOpEffect();

		@Override
		public void makeItSo(ManagedComponent component) {
			component.getLogger().debug(
					"CASTEffect: no need to do anything here, captain!",
					component.getLogAdditions());
		}

	}

}
