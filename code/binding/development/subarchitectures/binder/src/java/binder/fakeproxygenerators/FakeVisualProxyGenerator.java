package binder.fakeproxygenerators;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.Proxy;

public class FakeVisualProxyGenerator extends AbstractProxyGenerator {

	public void start () {
		Proxy newProxy = new Proxy();
		newProxy.proxyId = newDataID();
		newProxy.probExists = 0.78f;
		try {
		addToWorkingMemory(newProxy.proxyId, newProxy);
		log("new Proxy succesfully added to the binding working memory");
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
