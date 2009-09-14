package binder.fakeproxygenerators;

import java.util.Map;
import java.util.Random;

import binder.abstr.AbstractBindingPredictor;
import binder.autogen.core.Union;
import binder.autogen.specialentities.PhantomProxy;

abstract public class AbstractPhantomProxyGenerator extends AbstractBindingPredictor{

	protected int nbOfProxiesToCreate = 0;
	protected boolean reverted = false;

	protected boolean pauses = false;

	protected boolean deleteProxiesAfterBinding = true;
	
	
	@Override
	public void configure(Map<String, String> _config) {

		if (_config.containsKey("--nbproxies")) {
			nbOfProxiesToCreate = Integer.parseInt(_config.get("--nbproxies"));
		} 
		if (_config.containsKey("--reverted")) {
			reverted = Boolean.parseBoolean(_config.get("--reverted"));
		} 
		if (_config.containsKey("--pauses")) {
			pauses = Boolean.parseBoolean(_config.get("--pauses"));
		} 
		if (_config.containsKey("--deleteproxies")) {
			deleteProxiesAfterBinding = Boolean.parseBoolean(_config.get("--deleteproxies"));
		}
	}
	
	
	
	protected void randomPrediction() {

		Random rand = new Random();
		if (nbOfProxiesToCreate > 0) {
			for (int i = 1 ; i < (nbOfProxiesToCreate +1) ; i++) {
				if (pauses) {
				sleepComponent(500 + rand.nextInt(2000));
				}
				PhantomProxy p;
				if (!reverted) {
					p = createPhantomProxy(i);
				}
				else {
					p = createPhantomProxy(nbOfProxiesToCreate-i+1);
				}
				Union u = getPredictedUnion(p, deleteProxiesAfterBinding);
				log("PREDICTED UNION: " + u.entityID);

			}	
		}
	}
	
	abstract public PhantomProxy createPhantomProxy(int nb);
	
}
