package manipulation.core.share.baseConnector;

import java.util.LinkedList;
import java.util.List;
import java.util.Stack;
import java.util.Vector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.MathException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.BorderPoint;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.ModelPoint;
import manipulation.core.share.types.Polarcoordinate;
import manipulation.core.share.types.Region;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.core.share.types.VisionModel;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

/**
 * Represent a collection of algorithms to calculate on a map
 * 
 * @author ttoenige
 * 
 */
public class MapAlgorithms {
	private Logger logger = Logger.getLogger(this.getClass());

	// TODO nicht hier, als config machen
	Vector3D bestInitDirection = new Vector3D(1, 0, 0);

	private MapConnector mapCon;
	private Manipulator manipulator;

	/**
	 * Constructor of the map algorithm collection
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public MapAlgorithms(Manipulator manipulator) {
		this.manipulator = manipulator;
		this.mapCon = manipulator.getMapConnector();
	}

	public Region updateSurroundingFromWorldCoordinates(Item item)
			throws MathException, ItemException {

		Vector3D pointIn = (Vector3D) item
				.getAttribute(PropertyName.WORLD_POSITION);

		Vector2D point = pointIn.forgetThirdDimension();

		List<Vector2D> innerPoints = new LinkedList<Vector2D>();
		List<Vector2D> border = new LinkedList<Vector2D>();
		List<BorderPoint> borderViewPoints = new LinkedList<BorderPoint>();

		Stack<Vector2D> mustDo = new Stack<Vector2D>();

		int width = mapCon.getMap().getxSize();
		int height = mapCon.getMap().getySize();
		int[][] labels = new int[width][height];

		int innerCount = 0;
		int borderCount = 0;

		Vector2D index = mapCon.getIndexFromWorldCoordinates(point);

		int x = (int) index.getX();
		int y = (int) index.getY();

		for (int h = 0; h < height; h++)
			for (int w = 0; w < width; w++) {

				labels[w][h] = -1;
			}

		if (labels[x][y] < 0) {

			mustDo.add(new Vector2D(x, y));
			labels[x][y] = 1;
			innerCount++;
		}

		while (mustDo.size() > 0) {
			Vector2D thisPoint = mustDo.get(0);
			mustDo.remove(0);
			for (int th = -1; th <= 1; th++) {
				for (int tw = -1; tw <= 1; tw++) {
					int rx = (int) thisPoint.getX() + tw;
					int ry = (int) thisPoint.getY() + th;

					if ((rx < 0) || (ry < 0) || (ry >= height) || (rx >= width))
						continue;
					if (labels[rx][ry] < 0)
						// if (mapCon.getDataFromIndex(new Point2D(rx, ry)) ==
						// mapCon
						// .getDataFromIndex(new Point2D(thisPoint.getX(),
						// thisPoint.getY()))) {
						if ((mapCon.getDataFromIndex(new Vector2D(rx, ry)) == 2)
								|| (mapCon
										.getDataFromIndex(new Vector2D(rx, ry)) == 1)) {
							mustDo.add(new Vector2D(rx, ry));
							labels[rx][ry] = 1;
							innerPoints.add(mapCon
									.getWorldFromIndex(new Vector2D(rx, ry)));
							// innerCount++;
						} else {
							boolean add = false;
							for (int th2 = -1; th2 <= 1; th2++) {
								for (int tw2 = -1; tw2 <= 1; tw2++) {
									int rx2 = (int) rx + tw2;
									int ry2 = (int) ry + th2;

									if (mapCon.getDataFromIndex(new Vector2D(
											rx2, ry2)) == 1) {
										add = true;
									}

								}

							}
							if (add) {
								border
										.add(mapCon
												.getWorldFromIndex(new Vector2D(
														rx, ry)));
								borderCount++;
							}
						}
				}
			}

		}
		if (!border.isEmpty()) {
			borderViewPoints.add(new BorderPoint(border.get(0)));

			for (int i = 0; i < border.size(); i++) {
				boolean addPoint = true;
				for (int j = 0; j < borderViewPoints.size(); j++) {

					if (borderViewPoints.get(j).getPoint().calculateDistance(
							border.get(i)) < manipulator.getConfiguration()
							.getReachingDistance()) {
						addPoint = false;
						continue;
					}
				}
				if (addPoint) {

					borderViewPoints.add(new BorderPoint(border.get(i)));
				}
				addPoint = true;
			}
		} else {
			borderViewPoints
					.add(new BorderPoint(pointIn.forgetThirdDimension()));
		}

		return new Region(innerPoints, borderViewPoints);
	}

	/**
	 * function to generate all viewpoints for a specified border point
	 * 
	 * @param centerPoint
	 *            corresponding border point
	 * @param bestDirection
	 *            best direction to look at the object
	 * @param stepsize
	 *            distance in front of the border point
	 * @param angleBetween
	 *            angle between the viewPoints in degrees
	 * @return list of viewpoint around the border point
	 * @throws MathException
	 */
	public List<ViewPoint> generateSurroundingPoints(Vector2D centerPoint,
			Vector2D bestDirection, double stepsize, double angleBetween)
			throws MathException {

		List<ViewPoint> pointsArround = new LinkedList<ViewPoint>();

		Polarcoordinate dirPol = MathOperation
				.getPolarCoordinate(bestDirection);

		double bestAngle = dirPol.getPhi();
		boolean firstTime = true;
		Vector2D bestPoint = null;

		while (dirPol.getPhi() - bestAngle < 2 * Math.PI - 0.001) {

			Vector2D newDirection = MathOperation
					.getCartesianCoordinate(dirPol);
			double xNew = centerPoint.getX() + (stepsize * newDirection.getX());
			double yNew = centerPoint.getY() + (stepsize * newDirection.getY());

			Vector2D newPoint = new Vector2D(xNew, yNew);

			if (firstTime) {
				bestPoint = newPoint;
				firstTime = false;
			}

			double error = newPoint.calculateDistance(bestPoint);
			BasePositionData data;

			double angle = MathOperation.getBaseAngleToWatch(centerPoint,
					newPoint);
			data = new BasePositionData(newPoint, angle);

			pointsArround.add(new ViewPoint(error, data));

			dirPol.setPhi(dirPol.getPhi()
					+ MathOperation.getRadiant(angleBetween));

		}

		return pointsArround;
	}

	public List<ViewPoint> generatePotentialViewPoints(Item item)
			throws ItemException, MathException {

		List<ViewPoint> viewPoints = new LinkedList<ViewPoint>();
		List<BorderPoint> borderViewPoints = ((Region) item
				.getAttribute(PropertyName.SURROUNDING)).getBorderPoints();

		Vector3D itemPos = ((Vector3D) item
				.getAttribute(PropertyName.WORLD_POSITION));

		for (BorderPoint borderViewPoint : borderViewPoints) {
			List<ViewPoint> generatedViewpoint = generateSurroundingPoints(
					borderViewPoint.getPoint(), MathOperation.getDirection(
							itemPos.forgetThirdDimension(), borderViewPoint
									.getPoint()), manipulator
							.getConfiguration().getDistanceInFrontDesk(), 370);

			for (ViewPoint viewPoint : generatedViewpoint) {

				if ((manipulator.getMapConnector().isPointFree(
						viewPoint.getPosition().getPoint()) || manipulator
						.getMapConnector().isPointUnknown(
								viewPoint.getPosition().getPoint()))
						&& (MathOperation.getDistance(itemPos
								.forgetThirdDimension(), viewPoint
								.getPosition().getPoint()) < manipulator
								.getConfiguration()
								.getMaxDistanceForViewPoints())) {
					viewPoints.add(viewPoint);
				}
			}
		}

		if (viewPoints.isEmpty()) {
			if (mapCon.isPointFree(itemPos.forgetThirdDimension())
					|| mapCon.isPointUnknown(itemPos.forgetThirdDimension())) {
				viewPoints = generateSurroundingPoints(
						itemPos.forgetThirdDimension(),
						new Vector2D(1, 0),
						manipulator.getConfiguration().getDistanceInFrontDesk(),
						45);
			}
		}

		List<ViewPoint> newViewPoints = new LinkedList<ViewPoint>();
		for (ViewPoint vp : viewPoints) {
			double error = MathOperation.getDistance(vp.getPosition()
					.getPoint(), itemPos.forgetThirdDimension());

			ViewPoint newViewPoint = new ViewPoint(error, vp.getPosition());

			newViewPoints.add(newViewPoint);
		}

		return newViewPoints;

	}

	public List<ViewPoint> updateRotationViewPoints(Item item)
			throws ItemException {
		List<ViewPoint> viewPoints = ((ViewPoints) item
				.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT)).getPoints();

		// Matrix objectRotationInRob = (Matrix) item
		// .getAttribute(PropertyName.ITEM_IN_ROB_ROTATION);
		//
		// Vector3D objectPositionInRob = (Vector3D) item
		// .getAttribute(PropertyName.ITEM_IN_ROB_POSITION);

		Matrix objectInWorldRotation = (Matrix) item
				.getAttribute(PropertyName.WORLD_ROTATION);

		Vector3D objectInWorld = (Vector3D) item
				.getAttribute(PropertyName.WORLD_POSITION);

		Vector2D bestDirection = MathOperation.getMatrixVectorMultiplication(
				objectInWorldRotation, bestInitDirection)
				.forgetThirdDimension().norm();

		List<ViewPoint> updatedViewPoints = new LinkedList<ViewPoint>();

		for (ViewPoint viewPoint : viewPoints) {
			Vector2D currentDirection = MathOperation.getDirection(viewPoint
					.getPosition().getPoint(), objectInWorld
					.forgetThirdDimension());

			double error = MathOperation.getDistance(currentDirection,
					bestDirection);

			ViewPoint updatedViewPoint = new ViewPoint(error, viewPoint
					.getPosition());

			updatedViewPoints.add(updatedViewPoint);
		}

		return updatedViewPoints;

	}

	public List<ViewPoint> generateRotationalViewPoints(Item item,
			BasePositionData currentPos) throws ItemException, MathException {
		List<ViewPoint> viewPoints = new LinkedList<ViewPoint>();

		Vector3D itemPos = ((Vector3D) item
				.getAttribute(PropertyName.WORLD_POSITION));

		// Matrix objectRotationInRob = (Matrix) item
		// .getAttribute(PropertyName.ITEM_IN_ROB_ROTATION);
		//
		// Vector3D objectPositionInRob = (Vector3D) item
		// .getAttribute(PropertyName.ITEM_IN_ROB_POSITION);
		//
		// Vector2D bestDirection = MathOperation.getMatrixVectorMultiplication(
		// objectRotationInRob,
		// objectPositionInRob).forgetThirdDimension().norm();

		Matrix itemRot = ((Matrix) item
				.getAttribute(PropertyName.WORLD_ROTATION));

		Vector2D bestDirection = MathOperation.getMatrixVectorMultiplication(
				itemRot, bestInitDirection).forgetThirdDimension().norm();

		double currDist = manipulator.getConfiguration()
				.getDistanceInFrontDesk();

		List<ViewPoint> generatedViewpoint = generateSurroundingPoints(itemPos
				.forgetThirdDimension(), bestDirection, currDist, 180);

		for (ViewPoint viewPoint : generatedViewpoint) {

			if ((manipulator.getMapConnector().isPointFree(
					viewPoint.getPosition().getPoint()) || manipulator
					.getMapConnector().isPointUnknown(
							viewPoint.getPosition().getPoint()))
					&& (MathOperation.getDistance(itemPos
							.forgetThirdDimension(), viewPoint.getPosition()
							.getPoint()) < manipulator.getConfiguration()
							.getMaxDistanceForViewPoints())) {
				viewPoints.add(viewPoint);
			}
		}

		return viewPoints;
	}

	public ViewPoint getBestViewPoint(List<ViewPoint> vps) {
		ViewPoint bestViewPoint = null;
		double smallestError = Double.MAX_VALUE;

		for (ViewPoint vp : vps) {
			if (mapCon.isPointFree(vp.getPosition().getPoint())
					|| mapCon.isPointUnknown(vp.getPosition().getPoint())) {
				if (vp.getError() < smallestError) {
					bestViewPoint = vp;
					smallestError = bestViewPoint.getError();
				}
			}

		}
		return bestViewPoint;
	}

	public ViewPoint getBestViewPoint(Item item) throws ViewPointException,
			ItemException {
		ViewPoint bestViewPoint = null;
		double smallestError = Double.MAX_VALUE;

		try {

			for (ViewPoint vp : ((ViewPoints) item
					.getAttribute(PropertyName.VIEW_POINTS)).getPoints()) {
				if (mapCon.isPointFree(vp.getPosition().getPoint())
						|| mapCon.isPointUnknown(vp.getPosition().getPoint())) {
					if (vp.getError() < smallestError
							&& mapCon.isPointFree(vp.getPosition().getPoint())) {
						bestViewPoint = vp;
						smallestError = bestViewPoint.getError();
					}
				}
			}
		} catch (IndexOutOfBoundsException e) {
			throw new ViewPointException(
					"Cannot find a viewPoint to this border point");
		}
		if (bestViewPoint == null) {
			throw new ViewPointException(
					"Cannot find a viewPoint to this border point");
		} else {
			return bestViewPoint;
		}
	}

	public Vector2D calculateGraspingDirection(Item item) throws ItemException {

		Vector<Vector2D> borderInWorldList2D = new Vector<Vector2D>();
		for (ModelPoint point : ((VisionModel) item
				.getAttribute(PropertyName.MODEL)).getModelPoints()) {

			// calculate camCoordinates
			Matrix itemInCamRotation = (Matrix) item
					.getAttribute(PropertyName.ITEM_IN_CAM_ROTATION);
			Vector3D itemInCamPosition = (Vector3D) item
					.getAttribute(PropertyName.ITEM_IN_CAM_POSITION);
			Vector3D borderInCam = MathOperation.getVectorAddition(
					MathOperation.getMatrixVectorMultiplication(
							itemInCamRotation, point.getPosition()),
					itemInCamPosition);

			Vector3D borderInRob = manipulator.getCalibrationConnector()
					.getCamPointInRob(borderInCam);

			Vector3D borderInWorld = manipulator.getBaseConnector()
					.getRobotToWorldTranslation(borderInRob);

			borderInWorldList2D.add(borderInWorld.forgetThirdDimension());
		}

		// linear regression
		Vector2D meanvalue = MathOperation.getMean(borderInWorldList2D);

		double temp1 = 0;
		double temp2 = 0;
		for (Vector2D vector2d : borderInWorldList2D) {
			temp1 += ((vector2d.getX() - meanvalue.getX()) * (vector2d.getY() - meanvalue
					.getY()));
			temp2 += Math.pow((vector2d.getX() - meanvalue.getX()), 2);
		}
		double a1 = temp1 / temp2;
		double a0 = meanvalue.getY() - a1 * meanvalue.getX();

		Vector2D direction = new Vector2D(1, a0 + a1);
		Vector2D directionNorm = direction.norm();

		return directionNorm;
	}

	public Vector3D calculateNearestPointToCam(Item item) throws ItemException {
		Vector<Vector3D> borderInCamList2D = new Vector<Vector3D>();
		for (ModelPoint point : ((VisionModel) item
				.getAttribute(PropertyName.MODEL)).getModelPoints()) {

			// calculate camCoordinates
			Matrix itemInCamRotation = (Matrix) item
					.getAttribute(PropertyName.ITEM_IN_CAM_ROTATION);
			Vector3D itemInCamPosition = (Vector3D) item
					.getAttribute(PropertyName.ITEM_IN_CAM_POSITION);
			Vector3D borderInCam = MathOperation.getVectorAddition(
					MathOperation.getMatrixVectorMultiplication(
							itemInCamRotation, point.getPosition()),
					itemInCamPosition);

			borderInCamList2D.add(borderInCam);
		}

		double lowestZvalue = Double.MAX_VALUE;
		Vector3D nearestPoint = null;
		Vector3D originPoint = new Vector3D(0, 0, 0);
		for (Vector3D vector3d : borderInCamList2D) {
			double tempDist = MathOperation.getDistance(vector3d, originPoint);
			if (tempDist < lowestZvalue) {
				lowestZvalue = tempDist;
				nearestPoint = vector3d;
			}
		}

		return nearestPoint;
	}
}