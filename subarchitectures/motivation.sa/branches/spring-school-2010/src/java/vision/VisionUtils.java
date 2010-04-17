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
	 * @return
	 */
	public static VisualObject newVisualObject() {
		VisualObject obj = new VisualObject(new cogx.Math.Pose3(newVector3(),
				new Matrix33(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d)),
				new String[0], 0d, new cogx.Math.Sphere3(newVector3(), 0d),
				CASTUtils.getTimeServer().getCASTTime(),
				new VisualObjectView[0], new GeometryModel(new Vertex[0],
						new Face[0]), "", 0d);
		return obj;
	}


	public static String toString(ColorRGB _rgb) {
		return CASTUtils.concatenate("[ColorRGB ",_rgb.r, ",",_rgb.g,",",_rgb.b,"]");
	}

}
