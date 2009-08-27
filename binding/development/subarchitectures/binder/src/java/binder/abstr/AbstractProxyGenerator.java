package binder.abstr;

import java.util.Map;
import java.util.Random;

import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import cast.architecture.ManagedComponent;

public abstract class AbstractProxyGenerator extends ManagedComponent {

	protected int nbOfProxiesToCreate = 0;
	protected boolean reverted = false;
	
	
	protected void addEntityToWM(PerceivedEntity entity) {

		try {
		addToWorkingMemory(entity.entityID, entity);
		log("new Proxy succesfully added to the binding working memory");
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--nbproxies")) {
			nbOfProxiesToCreate = Integer.parseInt(_config.get("--nbproxies"));
		} 
		if (_config.containsKey("--reverted")) {
			reverted = Boolean.parseBoolean(_config.get("--reverted"));
		} 
	}
	
	
	public void randomInsertion() {	
		Random rand = new Random();
		if (nbOfProxiesToCreate > 0) {
			for (int i = 1 ; i < (nbOfProxiesToCreate +1) ; i++) {
				sleepComponent(500 + rand.nextInt(1000));
				Proxy p;
				if (!reverted) {
					p = createProxy(i);
				}
				else {
					p = createProxy(nbOfProxiesToCreate-i+1);
				}
				addEntityToWM(p);

			}	
		}
	}
	
	
	protected abstract Proxy createProxy (int i) ;
	
}
