package cdsr.data;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import cdsr.objects.SensedObject;

/**
 * Bit of a hacky way of saving object lists
 * 
 * @author nah
 * 
 */
public abstract class SavedObjectLists {

	private final static double DESK_WIDTH_M = 0.6;
	private final static double DESK_HEIGHT_M = DESK_WIDTH_M;

	private final static double WHITEBOARD_WIDTH_M = 0.6;
	private final static double WHITEBOARD_HEIGHT_M = DESK_WIDTH_M;

	private final static Point2D.Double CLASSROOM_1_WHITEBOARD_CENTROID = new Point2D.Double(
			-1.12092, -0.141441);

	private final static Point2D.Double[] CLASSROOM_1_DESK_CENTROIDS = {
			new Point2D.Double(1.97145, 2.7791),
			new Point2D.Double(1.70179, 0.831157),
			new Point2D.Double(1.66127, -0.707407),
			new Point2D.Double(1.94947, -2.28477),
			new Point2D.Double(3.6619, -2.13064),
			new Point2D.Double(3.79395, -0.745701),
			new Point2D.Double(3.67022, 0.678557),
			new Point2D.Double(3.55527, 2.6688),
			new Point2D.Double(5.03304, 2.47684),
			new Point2D.Double(5.19891, 0.75118),
			new Point2D.Double(4.77626, -1.19775),
			new Point2D.Double(5.13868, -2.34805),
			new Point2D.Double(6.67284, -2.22963),
			new Point2D.Double(6.2986, -0.443185),
			new Point2D.Double(6.77839, 0.732012),
			new Point2D.Double(6.74733, 2.45601) };

	private final static Point2D.Double CLASSROOM_2_WHITEBOARD_CENTROID = new Point2D.Double(
			-0.977161, -0.00114718);
	private final static Point2D.Double CLASSROOM_2_LECTERN_CENTROID = new Point2D.Double(
			0.001142, -2.10697);

	private final static Point2D.Double[] CLASSROOM_2_DESK_CENTROIDS = {
			new Point2D.Double(1.74636, -0.990364),
			new Point2D.Double(1.72431, 0.478168),
			new Point2D.Double(1.73581, 2.39582),
			new Point2D.Double(3.52525, 2.0057),
			new Point2D.Double(3.53921, 0.402668),
			new Point2D.Double(3.4958, -1.03134),
			new Point2D.Double(4.96362, -0.988124),
			new Point2D.Double(5.08432, 0.553087),
			new Point2D.Double(4.9074, 2.17421),
			new Point2D.Double(6.47713, 1.73651),
			new Point2D.Double(6.4533, 0.565735),
			new Point2D.Double(6.50458, -1.19997),

	};

	private static final ArrayList<Line2D.Double> toBox(
			Point2D.Double _centroid, double _width, double _height) {
		// TODO assumes correctly aligned orientation
		ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>(4);

		double halfWidth = _width / 2;
		double halfHeight = _height / 2;

		lines.add(new Line2D.Double(_centroid.x - halfWidth, _centroid.y
				- halfHeight, _centroid.x + halfWidth, _centroid.y - halfHeight));
		lines.add(new Line2D.Double(_centroid.x - halfWidth, _centroid.y
				- halfHeight, _centroid.x - halfWidth, _centroid.y + halfHeight));
		lines.add(new Line2D.Double(_centroid.x + halfWidth, _centroid.y
				+ halfHeight, _centroid.x + halfWidth, _centroid.y - halfHeight));
		lines.add(new Line2D.Double(_centroid.x + halfWidth, _centroid.y
				+ halfHeight, _centroid.x - halfWidth, _centroid.y + halfHeight));

		return lines;
	}

	private static ArrayList<SensedObject> recreateClassroomObjects(
			Point2D.Double[] _deskCentroids,
			Point2D.Double _whiteboardCentroid, Point2D.Double _lecternCentroid) {
		ArrayList<SensedObject> objects = new ArrayList<SensedObject>(
				_deskCentroids.length + 1);

		int deskCounter = 0;
		for (Point2D.Double centroid : _deskCentroids) {
			objects.add(new SensedObject("desk" + ++deskCounter, toBox(
					centroid, DESK_WIDTH_M, DESK_HEIGHT_M), "desk"));
		}

		if (_whiteboardCentroid != null) {
			objects.add(new SensedObject("whiteboard1", toBox(
					_whiteboardCentroid, WHITEBOARD_WIDTH_M,
					WHITEBOARD_HEIGHT_M), "whiteboard"));
		}

		if (_lecternCentroid != null) {
			objects.add(new SensedObject("lectern1", toBox(_lecternCentroid,
					DESK_WIDTH_M, DESK_HEIGHT_M), "lectern"));
		}

		return objects;
	}

	/**
	 * Get the objects as used in classroom-1 in the AAAI FSS paper.
	 * 
	 * @return
	 */
	public static ArrayList<SensedObject> getClassroom1Objects() {
		// translation done on demand, assuming this is only called once per
		// run...
		return recreateClassroomObjects(CLASSROOM_1_DESK_CENTROIDS,
				CLASSROOM_1_WHITEBOARD_CENTROID, null);
	}
	
	
	/**
	 * Get the objects as used in classroom-1 in the AAAI FSS paper.
	 * 
	 * @return
	 */
	public static ArrayList<SensedObject> getClassroom2Objects() {
		// translation done on demand, assuming this is only called once per
		// run...
		return recreateClassroomObjects(CLASSROOM_2_DESK_CENTROIDS,
				CLASSROOM_2_WHITEBOARD_CENTROID, CLASSROOM_2_LECTERN_CENTROID);
	}
}
