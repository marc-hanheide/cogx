package castutils.viewer.plugins;

import java.util.Vector;

import SpatialProbabilities.JointProbabilityValue;
import SpatialProbabilities.ProbabilityDistribution;
import SpatialProbabilities.StringRandomVariableValue;

import comadata.ComaRoom;

public class ComaRoomInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ComaRoom r = (ComaRoom) iceObject;
		String helperString;
		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("roomId=" + r.roomId);
		extraInfo.add("seedPlaceInstance=" + r.seedPlaceInstance);
		helperString = "categories=( ";
		if (r.categories!=null) {
			if (r.categories.massFunction!=null) {
				for (JointProbabilityValue jpv : r.categories.massFunction) {
					StringRandomVariableValue rvv = (StringRandomVariableValue) jpv.variableValues[0];
					String c = rvv.value;
					float prob_c = jpv.probability;
					helperString = helperString + c + "(p=" + prob_c + ") ";
				}
			}
		}
		extraInfo.add(helperString + ")");
		helperString = "places=( ";
		for (long p : r.containedPlaceIds)
			helperString = helperString + p + " ";
		extraInfo.add(helperString + ")");
		return extraInfo;
	}

}
