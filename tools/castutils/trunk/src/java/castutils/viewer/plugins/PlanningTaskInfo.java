package castutils.viewer.plugins;

import java.util.Vector;

import autogen.Planner.PlanningTask;


public class PlanningTaskInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		PlanningTask m = (PlanningTask) iceObject;
		
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add("ID="+m.id);
		extraInfo.add("firstActionID="+m.firstActionID);
		extraInfo.add("goal="+m.goal);
		extraInfo.add(m.planningStatus.name());
		return extraInfo;
	}

}
