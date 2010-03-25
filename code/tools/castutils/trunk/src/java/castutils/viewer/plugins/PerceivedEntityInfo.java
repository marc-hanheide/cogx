package castutils.viewer.plugins;

import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.PerceivedEntity;


public class PerceivedEntityInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		PerceivedEntity m = (PerceivedEntity) iceObject;
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add(m.timeStamp);
		String fStr = "features: ";
		for (Feature f : m.features) {
			fStr = fStr + f.featlabel + " ";
		}
		extraInfo.add(m.probExists);
		extraInfo.add(fStr);
		
		return extraInfo;
	}

}
