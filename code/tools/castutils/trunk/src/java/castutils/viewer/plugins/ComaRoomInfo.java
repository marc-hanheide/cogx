package motivation.util.viewer.plugins;

import java.util.Vector;

import comadata.ComaRoom;

public class ComaRoomInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		ComaRoom r = (ComaRoom) iceObject;
		String helperString;
		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("roomId=" + r.roomId);
		extraInfo.add("seedPlaceInstance=" + r.seedPlaceInstance);
		helperString = "concepts=( ";
		for (String c : r.concepts)
			helperString = helperString + c + " ";
		extraInfo.add(helperString + ")");
		helperString = "places=( ";
		for (long p : r.containedPlaceIds)
			helperString = helperString + p + " ";
		extraInfo.add(helperString + ")");
		return extraInfo;
	}

}
