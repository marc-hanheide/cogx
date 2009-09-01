package binder.fakeproxygenerators;

import java.util.Map;
import java.util.Random;

import binder.abstr.BindingWorkingMemoryWriter;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import cast.architecture.ManagedComponent;

public abstract class AbstractProxyGenerator extends BindingWorkingMemoryWriter {

	protected int nbOfProxiesToCreate = 0;
	protected boolean reverted = false;
	
	

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
				addProxyToWM(p);

			}	
		}
	}
	
	
	protected abstract Proxy createProxy (int i) ;
	
}
