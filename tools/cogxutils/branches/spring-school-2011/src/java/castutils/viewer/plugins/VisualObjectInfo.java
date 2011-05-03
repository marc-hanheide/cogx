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

}
