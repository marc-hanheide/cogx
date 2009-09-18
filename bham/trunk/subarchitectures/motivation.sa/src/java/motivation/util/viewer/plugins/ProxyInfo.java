package motivation.util.viewer.plugins;

import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.Proxy;


public class ProxyInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Proxy m = (Proxy) iceObject;
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add(m.timeStamp);
		String fStr = "features: ";
		extraInfo.add(m.probExists);
		for (Feature f : m.features) {
			fStr = fStr + f.featlabel + " ";
		}
		extraInfo.add(fStr);
		extraInfo.add(m.origin.localDataType+": "+m.origin.subarchId + "::"+ m.origin.localDataId);
		
		return extraInfo;
	}

}
