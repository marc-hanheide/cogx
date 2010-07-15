package castutils.viewer.plugins;

import java.util.Vector;

import castutils.slice.WMMap;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class WMMapInfo implements Plugin {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		WMMap map = (WMMap) iceObject;
		extraInfo.add("size()="+map.map.size());
		return extraInfo;
	}

	// /**
	// * @param fv
	// * @return
	// */
	// public static String toString(FeatureValue fv) {
	// String featStr="*";
	// if (fv instanceof IntegerValue)
	// featStr=Integer.toString(((IntegerValue) fv).val);
	// if (fv instanceof PointerValue)
	// featStr=CASTUtils.toString(((PointerValue) fv).beliefId);
	// if (fv instanceof StringValue)
	// featStr=((StringValue) fv).val;
	// if (fv instanceof FloatValue)
	// featStr= Double.toString(((FloatValue) fv).val);
	// if (fv instanceof BooleanValue)
	// featStr=Boolean.toString(((BooleanValue) fv).val);
	// return featStr;
	// }

}
