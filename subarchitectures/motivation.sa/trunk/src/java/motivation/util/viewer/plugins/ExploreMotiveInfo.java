package motivation.util.viewer.plugins;

import java.util.Vector;

import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.util.CASTTimeUtil;


public class ExploreMotiveInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ExploreMotive m = (ExploreMotive) iceObject;
		Vector<Object> extraInfo=new Vector<Object>();
		extraInfo.add("tries="+m.tries);
		extraInfo.add("placeID="+m.placeID);
		extraInfo.add("rank="+m.rank);
		extraInfo.add(m.status.name());
		return extraInfo;
	}

}
