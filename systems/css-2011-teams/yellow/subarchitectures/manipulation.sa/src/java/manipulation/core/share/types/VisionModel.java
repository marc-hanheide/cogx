package manipulation.core.share.types;

import java.util.List;

/**
 * represent a vision model of an item
 * 
 * @author ttoenige
 * 
 */
public class VisionModel {
	List<ModelPoint> modelPoints;

	private boolean extremaCalculated = false;
	private boolean avgsCalculated = false;

	private double lowestX = Double.MAX_VALUE, lowestY = Double.MAX_VALUE,
			lowestZ = Double.MAX_VALUE;
	private double highestX = Double.MIN_VALUE, highestY = Double.MIN_VALUE,
			highestZ = Double.MIN_VALUE;
	private double avgX = Double.MIN_VALUE, avgY = Double.MIN_VALUE,
			avgZ = Double.MIN_VALUE;

	private void calculateExtrema() {
		for (ModelPoint mp : getModelPoints()) {
			Vector3D point = mp.getPosition();
			if (point.getX() < lowestX)
				lowestX = point.getX();
			if (point.getX() > highestX)
				highestX = point.getX();
			if (point.getY() < lowestY)
				lowestY = point.getY();
			if (point.getY() > highestY)
				highestY = point.getY();
			if (point.getZ() < lowestZ)
				lowestZ = point.getZ();
			if (point.getZ() > highestZ)
				highestZ = point.getZ();
		}

		extremaCalculated = true;
	}

	private void calculateAvgs() {
		double size = getModelPoints().size();

		double xTemp = 0, yTemp = 0, zTemp = 0;

		for (int i = 0; i < size; i++) {
			xTemp += getModelPoints().get(i).getPosition().getX();
			yTemp += getModelPoints().get(i).getPosition().getY();
			zTemp += getModelPoints().get(i).getPosition().getZ();
		}

		avgX = xTemp / size;
		avgY = yTemp / size;
		avgZ = zTemp / size;

		avgsCalculated = true;
	}

	/**
	 * constructor of the vision model
	 * 
	 * @param modelPoints
	 *            list of the corresponding model points
	 */
	public VisionModel(List<ModelPoint> modelPoints) {
		this.modelPoints = modelPoints;
	}

	/**
	 * 
	 * @return list of the corresponding model points
	 */
	public List<ModelPoint> getModelPoints() {
		return modelPoints;
	}

	/**
	 * set the list of the corresponding model points
	 * 
	 * @param modelPoints
	 *            new model points
	 */
	public void setModelPoints(List<ModelPoint> modelPoints) {
		this.modelPoints = modelPoints;
	}

	/**
	 * gets the highest x value
	 * 
	 * @return highest x value
	 */
	public double getHighestXValue() {
		if (!extremaCalculated)
			calculateExtrema();
		return highestX;
	}

	/**
	 * gets the lowest x value
	 * 
	 * @return lowest x value
	 */
	public double getLowestXValue() {
		if (!extremaCalculated)
			calculateExtrema();
		return lowestX;
	}

	/**
	 * gets the highest y value
	 * 
	 * @return highest y value
	 */
	public double getHighestYValue() {
		if (!extremaCalculated)
			calculateExtrema();
		return highestY;
	}

	/**
	 * gets the lowest y value
	 * 
	 * @return lowest y value
	 */
	public double getLowestYValue() {
		if (!extremaCalculated)
			calculateExtrema();
		return lowestY;
	}

	/**
	 * gets the highest z value
	 * 
	 * @return highest z value
	 */
	public double getHighestZValue() {
		if (!extremaCalculated)
			calculateExtrema();
		return highestZ;
	}

	/**
	 * gets the lowest z value
	 * 
	 * @return lowest z value
	 */
	public double getLowestZValue() {
		if (!extremaCalculated)
			calculateExtrema();
		return lowestZ;
	}

	/**
	 * gets the average x value
	 * 
	 * @return average x value
	 */
	public double getAvgXvalue() {
		if (!avgsCalculated) {
			calculateAvgs();
		}
		return avgX;
	}

	/**
	 * gets the average y value
	 * 
	 * @return average y value
	 */
	public double getAvgYvalue() {
		if (!avgsCalculated) {
			calculateAvgs();
		}
		return avgY;
	}

	/**
	 * gets the average z value
	 * 
	 * @return average z value
	 */
	public double getAvgZvalue() {
		if (!avgsCalculated) {
			calculateAvgs();
		}
		return avgZ;
	}
}
