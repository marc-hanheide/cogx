package vision;

import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import blobfinder.ColorRGB;
import cast.core.CASTUtils;
import cogx.Math.Matrix33;
import cogx.Math.Vector3;

public class VisionUtils {

	/**
	 * @return
	 */
	public static Vector3 newVector3() {
		return new Vector3(0d, 0d, 0d);
	}

	/**
	 * Produces non-null default object
	 * 
	 * @return
	 */
	public static VisualObject newVisualObject() {

		// public VisualObject(cogx.Math.Pose3 pose, String[] patchIds, double
		// detectionConfidence, cogx.Math.Sphere3 boundingSphere,
		// cast.cdl.CASTTime time, String componentID, VisualObjectView[] views,
		// GeometryModel model, String label, double labelConfidence, double
		// salience, int[] labels, double[] distribution, String[] identLabels,
		// double[] identDistrib, double identGain, double identAmbiguity,
		// String[] colorLabels, double[] colorDistrib, double colorGain, double
		// colorAmbiguity, String[] shapeLabels, double[] shapeDistrib, double
		// shapeGain, double shapeAmbiguity, String protoObjectID)

		VisualObject obj = new VisualObject(new cogx.Math.Pose3(newVector3(),
				new Matrix33(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d)),
				new String[0], 0d, new cogx.Math.Sphere3(newVector3(), 0d),
				CASTUtils.getTimeServer().getCASTTime(), null,
				new VisualObjectView[0], new GeometryModel(new Vertex[0],
						new Face[0]),
				0d, new String[0], new double[0], 0, 0, new String[0], 
				new double[0], new double[0], 0, 0, new String[0], new double[0],  new double[0], 0, 0, "", "");
		return obj;
	}

	public static String toString(ColorRGB _rgb) {
		return CASTUtils.concatenate("[ColorRGB ", _rgb.r, ",", _rgb.g, ",",
				_rgb.b, "]");
	}

}
