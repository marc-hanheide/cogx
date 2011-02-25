package manipulation.core.bham.camConnector;

import manipulation.core.bham.converter.BhamConverter;
import manipulation.core.bham.simulationConnector.BhamSimulationConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.ItemName;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.bham.BhamRunner;

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
public class BhamBlortConnector implements CamConnector {

	private Logger logger = Logger.getLogger(this.getClass());
	
	private Manipulator manipulator;

	private boolean objectFound = false;

	/**
	 * constructor of the BLORT visual system
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public BhamBlortConnector(Manipulator manipulator) {
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
		try {

			VisualObject changedObject = ((BhamRunner) manipulator.getRunner())
					.getMemoryEntry(_wmc.address, VisualObject.class);

			Vector3D camPoint = new Vector3D(changedObject.pose.pos.x,
					changedObject.pose.pos.y, changedObject.pose.pos.z);

			Matrix camRot = BhamConverter
					.convBlortToMatrix(changedObject.pose.rot);

			// TODO richtige Item updaten nicht einfach erst beste
			if (!manipulator.getItemMemory().getFirstGraspItem()
					.getAllAtributeKeys().contains(PropertyName.MODEL)) {
				manipulator
						.getItemMemory()
						.addItemModel(
								manipulator.getItemMemory().getFirstGraspItem(),
								BhamConverter
										.convBlortGeomModelToVisionModel(changedObject.model));
			}

			if (changedObject.identDistrib[0] > 0.08) {
				manipulator.getItemMemory().updatePosition(
						manipulator.getItemMemory().getFirstGraspItem(),
						camPoint, camRot, manipulator, false);
				objectFound = true;
			} else {
				objectFound = false;
			}

			synchronized (this) {
				notifyAll();
			}

		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (UnknownSubarchitectureException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean recognizeTrackItem(Item item) throws ExternalMemoryException {
		Recognizer3DCommand recognizeCommand = new Recognizer3DCommand();
		recognizeCommand.cmd = Recognizer3DCommandType.RECOGNIZE;

		String id = ((BhamRunner) manipulator.getRunner()).newDataID();

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
			default:
				logger.error("don't know the item");
				break;
			}
		} catch (ItemException e1) {
			logger.error(e1);
		}

		try {
			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id,
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

		String id2 = ((BhamRunner) manipulator.getRunner()).newDataID();

		try {
			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id2,
					recognizeCommand);
		} catch (AlreadyExistsOnWMException e) {
			throw new ExternalMemoryException(e.message);
		}

		// TrackingCommand trackCom = new TrackingCommand();
		// trackCom.cmd = TrackingCommandType.RELEASEMODELS;
		//
		// String id = ((BhamRunner) manipulator.getRunner()).newDataID();
		//
		// try {
		// ((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id,
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

		String id = ((BhamRunner) manipulator.getRunner()).newDataID();

		try {
			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id,
					trackCom);
		} catch (CASTException e) {
			throw new ExternalMemoryException(e.message);

		}

		((BhamSimulationConnector) manipulator.getSimulationConnector())
				.startItemThread();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopTracking() throws ExternalMemoryException {
		logger.debug("stop tracking");

		((BhamSimulationConnector) manipulator.getSimulationConnector())
				.stopItemThread();

		TrackingCommand trackCom = new TrackingCommand();
		trackCom.cmd = TrackingCommandType.STOP;

		String id = ((BhamRunner) manipulator.getRunner()).newDataID();

		try {
			((BhamRunner) manipulator.getRunner()).addToWorkingMemory(id,
					trackCom);
		} catch (CASTException e) {
			throw new ExternalMemoryException(e.message);
		}
	}

}
