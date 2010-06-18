package cogxutils.viewer.plugins;

import java.util.Vector;

import motivation.slice.Motive;
import castutils.CASTTimeUtil;
import castutils.viewer.plugins.Plugin;


public class MotiveInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Motive m = (Motive) iceObject;
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add("tries="+m.tries);
		extraInfo.add(CASTTimeUtil.diff(m.updated, m.created));
		extraInfo.add(m.referenceEntry.subarchitecture + "::"+m.referenceEntry.id);
		extraInfo.add(m.status.name());
		return extraInfo;
	}

}
