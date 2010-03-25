package motivation.util.viewer.plugins;

import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.RelationProxy;

public class RelationProxyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		if (iceObject instanceof RelationProxy) {
			RelationProxy m = (RelationProxy) iceObject;
			String helperString = "features=( ";
			for (Feature c : m.features)
				helperString = helperString + c.featlabel + " ";
			extraInfo.add(helperString + ")");

			extraInfo.add("src="+((AddressValue) m.source.alternativeValues[0]).val);
			extraInfo.add("dst="+((AddressValue) m.target.alternativeValues[0]).val);
		}
		return extraInfo;
	}

}
