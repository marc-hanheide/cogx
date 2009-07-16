package binder.abstr;

import binder.autogen.core.PerceivedEntity;
import cast.architecture.ManagedComponent;

public class AbstractProxyGenerator extends ManagedComponent {

	
	protected void addEntityToWM(PerceivedEntity entity) {

		try {
		addToWorkingMemory(entity.entityID, entity);
		log("new Proxy succesfully added to the binding working memory");
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
