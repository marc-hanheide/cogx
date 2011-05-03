package castutils.viewer.plugins;

import java.util.Vector;

import VisionData.VisualObject;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class VisualObjectInfo extends DefaultXMLInfo {

	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		VisualObject vo = (VisualObject) iceObject;
		extraInfo.add(DefaultXMLInfo.toXML(vo.pose));
		extraInfo.add(DefaultXMLInfo.toXML(vo.identLabels));
		extraInfo.add(DefaultXMLInfo.toXML(vo.identDistrib));
		extraInfo.add(DefaultXMLInfo.toXML(vo.detectionConfidence));
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
