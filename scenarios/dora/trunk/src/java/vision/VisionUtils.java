package vision;

import org.apache.log4j.Logger;

import mathlib.Functions;
import Video.CameraParameters;
import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import blobfinder.ColorRGB;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cogx.Math.Matrix33;
import cogx.Math.Vector2;
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

		VisualObject obj = new VisualObject();

		return obj;
	}

	public static String toString(ColorRGB _rgb) {
		return CASTUtils.concatenate("[ColorRGB ", _rgb.r, ",", _rgb.g, ",",
				_rgb.b, "]");
	}

	/**
	 * check if a 3D point (in robot CS) is visible in the camera
	 * 
	 * @param cam
	 * @param w
	 * @return returns true if a 3D point is visible in the frame of the camera
	 */
	static public boolean isVisible(CameraParameters cam, Vector3 w) {
		Vector2 imgCoords = projectPoint(cam, w);
		Logger.getLogger(VisionUtils.class).debug(
				"projected coords=[" + imgCoords.x + ", " + imgCoords.y + "]");
		return (imgCoords.x >= 0 && imgCoords.x < cam.width && imgCoords.y >= 0 && imgCoords.y < cam.height);
	}

	/**
	 * project a 3D point (in robot CS) into the image plane
	 * 
	 * @param cam
	 * @param w
	 * @return the 2D coordinates of the projected point
	 */
	static public Vector2 projectPoint(CameraParameters cam, Vector3 w) {
		Vector3 p = Functions.transformInverse(cam.pose, w);
		Logger.getLogger(VisionUtils.class).debug(
				"in cam coords: [" + p.x + ", " + p.y + ", " + p.z + "]");
		Logger.getLogger(VisionUtils.class).debug(
				"cam params: [" + cam.fx + ", " + cam.fy + ", " + cam.cx + ", "
						+ cam.cy + "]");

		return new Vector2(cam.fx * p.x / p.z + cam.cx, cam.fy * p.y / p.z
				+ cam.cy);
	}

}
