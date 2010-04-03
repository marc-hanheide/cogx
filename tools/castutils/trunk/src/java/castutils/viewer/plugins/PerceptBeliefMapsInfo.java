package castutils.viewer.plugins;

import java.util.Vector;

import binder.autogen.beliefs.Belief;
import binder.autogen.perceptmanagement.PerceptBeliefMaps;

import Ice.ObjectImpl;

public class PerceptBeliefMapsInfo implements Plugin {

	@Override
	public Vector<Object> toVector(ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		PerceptBeliefMaps pbm = (PerceptBeliefMaps) iceObject;
		if (pbm != null) {
			extraInfo.add("percept2Belief size: " + pbm.percept2Belief.size());
		}
		return extraInfo;
	}

}
