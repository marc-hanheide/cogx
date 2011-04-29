package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import motivation.slice.ExploreMotive;


public class ExploreMotiveInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ExploreMotive m = (ExploreMotive) iceObject;
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add("goal: " + m.goal.goalString);
		extraInfo.add("costs="+m.costs+" plannedCosts="+m.plannedCosts);
		extraInfo.add("info-gain="+m.informationGain + ", rank="+m.rank);
		extraInfo.add(m.status.name());
		return extraInfo;
	}

}
