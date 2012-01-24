package manipulation.core.cogx.camConnector;

import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.cogx.CogXRunner;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryChange;

/**
 * represent a connector to the BLORT visual system
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXBlortConnector implements CamConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * constructor of the BLORT visual system connector
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public CogXBlortConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * listener function on the CAST working memory, recognizes if a visual
	 * object is changed in the memory
	 * 
	 * @param _wmc
	 *            relevant working memory change
	 */
	public void visualObjectChanged(WorkingMemoryChange _wmc) {

		VisualObject changedObject = null;
		try {
			changedObject = ((CogXRunner) manipulator.getRunner())
					.getMemoryEntry(_wmc.address, VisualObject.class);
		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (UnknownSubarchitectureException e) {
			logger.error(e);
		}

		Vector3D camPoint = new Vector3D(changedObject.pose.pos.x,
				changedObject.pose.pos.y, changedObject.pose.pos.z);

		Matrix camRot = CogXConverter.convBlortToMatrix(changedObject.pose.rot);

		double bestRecValue = 0;
		String bestLabel = "";

		for (int i = 0; i < changedObject.identLabels.length; i++) {
			if (!changedObject.identLabels[i].equals("unknown")) {
				if (changedObject.identDistrib[i] > bestRecValue) {
					bestRecValue = changedObject.identDistrib[i];
					bestLabel = changedObject.identLabels[i];
				}
			}
		}

		if (!bestLabel.equals("")) {
			boolean alreadyInMem = false;
			Item currentItem = null;
			for (Item item : manipulator.getItemMemory().getItemList()) {
				try {
					if (((String) item.getAttribute(PropertyName.BLORT_NAME))
							.equals(bestLabel)) {
						alreadyInMem = true;
						currentItem = item;
						break;
					}
				} catch (ItemException e) {
					logger.debug(e);
				}
			}

			if (alreadyInMem) {
				try {
					manipulator.getItemMemory().updatePosition(currentItem,
							camPoint, camRot, manipulator, false);
				} catch (InternalMemoryException e) {
					logger.error(e);
				}
			} else {
				logger.debug("Added obstacle with label " + bestLabel
						+ "to arm planning");
				Item newItem = new Item();
				newItem.setAttribute(PropertyName.BLORT_NAME, bestLabel);
				newItem.setAttribute(PropertyName.WMA_ADDRESS, _wmc.address);
				newItem.setAttribute(PropertyName.WORLD_POSITION, new Vector3D(
						Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
				newItem.setAttribute(PropertyName.WORLD_ROTATION, new Matrix(1,
						0, 0, 0, 1, 0, 0, 0, 1));

				manipulator.getItemMemory().addItemToQueue(newItem);

				try {
					manipulator.getItemMemory().updatePosition(newItem,
							camPoint, camRot, manipulator, false);
				} catch (InternalMemoryException e) {
					logger.error(e);
				}

				try {
					manipulator
							.getItemMemory()
							.addItemModel(
									newItem,
									CogXConverter
											.convBlortGeomModelToVisionModel(changedObject.model));
				} catch (InternalMemoryException e) {
					logger.error(e);
				}
			}
		} else {
			logger.debug("Recognition value: " + bestRecValue
					+ "! Will not add anything to the arm planning.");
		}
	}

	public void visualObjectDelete(WorkingMemoryChange _wmc) {
		VisualObject changedObject = null;
		try {
			changedObject = ((CogXRunner) manipulator.getRunner())
					.getMemoryEntry(_wmc.address, VisualObject.class);
		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (UnknownSubarchitectureException e) {
			logger.error(e);
		}

		double bestRecValue = 0;
		String bestLabel = "";

		for (int i = 0; i < changedObject.identLabels.length; i++) {
			if (!changedObject.identLabels[i].equals("unknown")) {
				if (changedObject.identDistrib[i] > bestRecValue) {
					bestRecValue = changedObject.identDistrib[i];
					bestLabel = changedObject.identLabels[i];
				}
			}
		}

		if (!bestLabel.equals("")) {
			Item currentItem = null;
			for (Item item : manipulator.getItemMemory().getItemList()) {
				try {
					if (((String) item.getAttribute(PropertyName.BLORT_NAME))
							.equals(bestLabel)) {
						currentItem = item;
						break;
					}
				} catch (ItemException e) {
					logger.debug(e);
				}
			}

			try {
				manipulator.getItemMemory().deleteItem(currentItem);
			} catch (InternalMemoryException e) {
				logger.error("Cannot delete the item");
			}

		} else {
			logger.debug("Recognition value: " + bestRecValue
					+ "! Will not delete anything.");
		}
	}
}
