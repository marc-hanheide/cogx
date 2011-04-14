package manipulation.core.cogx.camConnector;

import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.ItemName;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.cogx.CogXRunner;

import org.apache.log4j.Logger;

import VisionData.Recognizer3DCommand;
import VisionData.Recognizer3DCommandType;
import VisionData.TrackingCommand;
import VisionData.TrackingCommandType;
import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryChange;

/**
 * represent a connector to the BLORT visual system
 * 
 * @author ttoenige
 * 
 */
public class CogXBlortConnector implements CamConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	private boolean objectFound = false;

	/**
	 * constructor of the BLORT visual system
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

		logger.error("OBJ-POSE: " + camPoint);
		
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

		// logger.error("BEST LABEL: " + bestLabel);
		// logger.error("BEST VALUE: " + bestRecValue);

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
				logger.error("Added obstacle with label " + bestLabel
						+ "to arm planning");
				Item newItem = new Item();
				newItem.setAttribute(PropertyName.BLORT_NAME, bestLabel);
				newItem.setAttribute(PropertyName.WMA_ADDRESS, _wmc.address);
				newItem.setAttribute(PropertyName.INTENTION,
						ItemIntention.AVOID_ME);
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

			if (bestRecValue > 0.08) {
				objectFound = true;
			} else {
				objectFound = false;
			}
		} else {
			logger.error("Recognition value: " + bestRecValue
					+ "! Will not add anything to the arm planning.");
		}

		synchronized (this) {
			notifyAll();
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean recognizeTrackItem(Item item) throws ExternalMemoryException {
		Recognizer3DCommand recognizeCommand = new Recognizer3DCommand();
		recognizeCommand.cmd = Recognizer3DCommandType.RECOGNIZE;

		String id = ((CogXRunner) manipulator.getRunner()).newDataID();

		try {
			switch (((ItemName) item.getAttribute(PropertyName.NAME))) {
			case FROSTIES_SMALL:
				// TODO iwo anders machen?
				recognizeCommand.label = "frosties_small";
				logger.debug("try to find frosties_small");
				break;
			case SOUP:
				recognizeCommand.label = "soup";
				logger.debug("try to find soup");
				break;
			case PRINGLES:
				recognizeCommand.label = "pringles";
				logger.debug("try to find pringles");
				break;
			case GREEN_TEA:
				recognizeCommand.label = "green_tea";
				logger.debug("try to find green_tea");
				break;
			default:
				logger.error("don't know the item");
				break;
			}
		} catch (ItemException e1) {
			logger.error(e1);
		}

		try {
			((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
					recognizeCommand);
		} catch (CASTException e) {
			throw new ExternalMemoryException(e.message);
		}

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}

		if (objectFound)
			startTracking();

		// TODO doofe loesung
		try {
			Thread.sleep(3000);
		} catch (InterruptedException e) {
			logger.error(e);
		}

		logger.error("success Funkt: " + objectFound);
		return objectFound;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void resetTracker() throws ExternalMemoryException {
		logger.debug("reset tracking");

		objectFound = false;

		stopTracking();

		Recognizer3DCommand recognizeCommand = new Recognizer3DCommand();
		recognizeCommand.cmd = Recognizer3DCommandType.RECSTOP;

		String id2 = ((CogXRunner) manipulator.getRunner()).newDataID();

		try {
			((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id2,
					recognizeCommand);
		} catch (AlreadyExistsOnWMException e) {
			throw new ExternalMemoryException(e.message);
		}

		// TrackingCommand trackCom = new TrackingCommand();
		// trackCom.cmd = TrackingCommandType.RELEASEMODELS;
		//
		// String id = ((CogXRunner) manipulator.getRunner()).newDataID();
		//
		// try {
		// ((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
		// trackCom);
		// } catch (CASTException e) {
		// throw new ExternalMemoryException(e.message);
		// }

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void startTracking() throws ExternalMemoryException {
		logger.debug("start tracking");

		TrackingCommand trackCom = new TrackingCommand();
		trackCom.cmd = TrackingCommandType.START;

		String id = ((CogXRunner) manipulator.getRunner()).newDataID();

		try {
			((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
					trackCom);
		} catch (CASTException e) {
			throw new ExternalMemoryException(e.message);

		}

		// ((CogXSimulationConnector) manipulator.getSimulationConnector())
		// .startItemThread();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopTracking() throws ExternalMemoryException {
		logger.debug("stop tracking");

		// ((CogXSimulationConnector) manipulator.getSimulationConnector())
		// .stopItemThread();

		TrackingCommand trackCom = new TrackingCommand();
		trackCom.cmd = TrackingCommandType.STOP;

		String id = ((CogXRunner) manipulator.getRunner()).newDataID();

		try {
			((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
					trackCom);
		} catch (CASTException e) {
			throw new ExternalMemoryException(e.message);
		}
	}

}
