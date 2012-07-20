package castutils.viewer.plugins;

import java.util.Vector;

import castutils.viewer.plugins.Plugin;

import autogen.Planner.Action;

public class ActionInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		if (iceObject instanceof Action) {
			Action m = (Action) iceObject;
			extraInfo.add("ID: " + m.taskID);
			extraInfo.add(m.fullName);
			extraInfo.add(m.status.name());
		}
		return extraInfo;
	}

}
