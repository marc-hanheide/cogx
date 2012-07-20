package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import autogen.Planner.PlanningTask;


public class PlanningTaskInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		PlanningTask m = (PlanningTask) iceObject;
		
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add("ID="+m.id);
		extraInfo.add("firstActionID="+m.firstActionID);
		extraInfo.add("first goal="+m.goals[0].goalString);
		extraInfo.add(m.planningStatus.name());
		return extraInfo;
	}

}
