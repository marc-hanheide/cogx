package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import SpatialProperties.AgentProperty;
import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.FloatValue;
import SpatialProperties.IntegerValue;

public class AgentPropertyInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		AgentProperty gpp = (AgentProperty) iceObject;

		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("agentID=" + gpp.agentID);
		if (gpp.distribution != null
				&& ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value instanceof FloatValue) {

			double val = ((FloatValue) ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value).value;
			extraInfo.add("val=" + val);
		} else if (gpp.distribution != null
				&& ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value instanceof IntegerValue) {

			long val = ((IntegerValue) ((DiscreteProbabilityDistribution) gpp.distribution).data[0].value).value;
			extraInfo.add("val=" + val);
		}

		return extraInfo;
	}

}
