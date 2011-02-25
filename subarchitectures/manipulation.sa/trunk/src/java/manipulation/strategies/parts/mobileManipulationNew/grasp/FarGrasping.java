package manipulation.strategies.parts.mobileManipulationNew.grasp;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to reach a position in front of an object with the arm
 * 
 * @author ttoenige
 * 
 */
public class FarGrasping extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private Vector3D currentGoalPosition = new Vector3D(Double.MAX_VALUE,
			Double.MAX_VALUE, Double.MAX_VALUE);

	private Vector3D lastnewItemPosition;

	private Vector3D newItemPosition = new Vector3D(0, 0, 0);

	int notMovingcounter = 0;

	/**
	 * constructor for the grasping part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public FarGrasping(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FAR_GRASPING);
	}

	private void alwaysGrasp() {

		Vector3D currentArmPos = null;
		try {
			currentArmPos = getManipulator().getArmConnector()
					.getCurrentPosition();
		} catch (ManipulatorException e) {
			logger.error("1");
			logger.error(e);
		}

		double posInFront = 0.1;

		Vector3D direction = MathOperation.getDirection(currentArmPos,
				currentGoalPosition);

		logger.error("Grasp the item!");

		Matrix rotation1 = MathOperation.getRotationAroundX(MathOperation
				.getRadiant(0));

		Matrix rotation2 = MathOperation.getRotationAroundY(MathOperation
				.getRadiant(0));

		Matrix rotation3 = MathOperation.getRotationAroundZ(MathOperation
				.getRadiant(-90));

		Matrix greifRotation = MathOperation.getMatrixMatrixMultiplication(
				MathOperation.getMatrixMatrixMultiplication(rotation1,
						rotation2), rotation3);

		// ((BhamSimulationConnector)getManipulator().getSimulationConnector()).addBox(greifRotation);

		Vector3D goalWithDistance = new Vector3D(
				(currentGoalPosition.getX() - posInFront * direction.getX()),
				currentGoalPosition.getY() - posInFront * direction.getY(),
				currentGoalPosition.getZ());

		ArmError armError = null;
		try {
			armError = getManipulator().getArmConnector().getPosError(
					goalWithDistance, greifRotation);
		} catch (ManipulatorException e) {
			logger.error("2");
			logger.error(e);
		}

		logger.error(MathOperation.getEuclDistance(armError.getPoseError()));

		// if (MathOperation.getEuclDistance(armError.getPoseError()) < 0.0001)
		// {
		logger.error("GREIFE");
		try {
			getManipulator().getArmConnector().reach(goalWithDistance,
					greifRotation);
		} catch (ManipulatorException e) {
			logger.error("3");
			logger.error(e);

			logger.error("greife nicht test!");
			stopTracking();

			try {
				if (getManipulator().getSimulationConnector()
						.removeGraspingBasePoint(
								getManipulator().getSimulationConnector()
										.getBestGraspingBasePoint())) {
					logger.error("another try");

					setNextPartName(PartName.GO_TO_BEST_GRASPING_POINT);
					synchronized (this) {
						notifyAll();
					}
				} else {
					logger.error("No more Grasping point -> go on");

					Item firstGraspItem = null;
					try {
						firstGraspItem = getManipulator().getItemMemory()
								.getFirstGraspItem();
					} catch (InternalMemoryException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}

					try {
						if (!((ViewPoints) firstGraspItem
								.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT))
								.getPoints().isEmpty()) {

							getManipulator()
									.getItemMemory()
									.deleteRotationalVP(
											firstGraspItem,
											(ViewPoint) firstGraspItem
													.getAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT));

							setNextPartName(PartName.GO_TO_BEST_ROTATIONAL_POINT);
							synchronized (this) {
								notifyAll();
							}
						} else {
							getManipulator()
									.getItemMemory()
									.removeViewPoint(
											firstGraspItem,
											(ViewPoint) firstGraspItem
													.getAttribute(PropertyName.BEST_VIEW_POINT),
											getManipulator());

							setNextPartName(PartName.FAR_APPROACH);

							synchronized (this) {
								notifyAll();
							}

						}
					} catch (ItemException e1) {
						logger.error(e1);
					} catch (InternalMemoryException e1) {
						logger.error(e1);
					}

				}
			} catch (ViewPointException e1) {
				logger.error(e1);
			}
		}
	}

	private void stopTracking() {
		try {
			getManipulator().getCamConnector().resetTracker();
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getItemMemory().addObserver(this);

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}

		logger.debug("we go on!");
		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		getManipulator().getItemMemory().deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));
	}

	/**
	 * 
	 * {@inheritDoc}
	 */
	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof ItemMemory) {
			if (arg instanceof Vector3D) {

				try {
					Item newItem = null;
					try {
						newItem = ((ItemMemory) observable).getFirstGraspItem();
					} catch (InternalMemoryException e) {
						logger.error(e);
					}
					lastnewItemPosition = newItemPosition;

					newItemPosition = ((Vector3D) newItem
							.getAttribute(PropertyName.WORLD_POSITION));

					if (MathOperation.getDistance(currentGoalPosition,
							newItemPosition) > 0.05) {

						if (!(MathOperation.getDistance(lastnewItemPosition,
								newItemPosition) > 0.002)) {
							currentGoalPosition = newItemPosition;
							logger.error("not moving -> grasping");

							alwaysGrasp();
						}
					}

					if (getManipulator().getArmConnector().isReached()
							&& !getManipulator().getArmConnector().isHome()) {
						logger.error("Arm Position erreicht->feines greifen");

						setNextPartName(PartName.FINE_GRASPING);

						synchronized (this) {
							notifyAll();
						}
					}
				} catch (ItemException e1) {
					logger.error(e1);
					stopTracking();
					setNextPartName(PartName.FAR_APPROACH);
					synchronized (this) {
						notifyAll();
					}
				}

			}
		}
	}
}
