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

}
