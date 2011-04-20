package manipulation.muster.strategies.parts.calibration;

import java.util.LinkedList;
import java.util.List;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.types.BorderPoint;
import manipulation.muster.core.share.types.Matrix;
import manipulation.muster.core.share.types.PTZPosition;
import manipulation.muster.core.share.types.Region;
import manipulation.muster.core.share.types.Vector2D;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.core.share.types.ViewPoint;
import manipulation.muster.core.share.types.ViewPoints;
import manipulation.muster.itemMemory.Item;
import manipulation.muster.itemMemory.Item.ItemIntention;
import manipulation.muster.itemMemory.Item.ItemName;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;
import manipulation.muster.visualisation.CalibrationGUI;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to calibrate the arm to the camera
 * 
 * @author ttoenige
 * 
 */
public class CalibrationPart extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * constructor for the calibration part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public CalibrationPart(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.CALIBRATION);

		this.manipulator = manipulator;

		manipulator.getPanTiltConnector().setPoseDeg(
				new PTZPosition(0, getManipulator().getConfiguration()
						.getFixedtiltAngle()));

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		logger.error("Change to next part");

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("Execture Calibration Part");

		Item item = new Item();
		item.setAttribute(PropertyName.NAME, ItemName.FROSTIES_SMALL);
		item.setAttribute(PropertyName.WORLD_POSITION, new Vector3D(1, 1, 1));
		item.setAttribute(PropertyName.WORLD_ROTATION, new Matrix(1, 0, 0, 0,
				1, 0, 0, 0, 1));
		item.setAttribute(PropertyName.INTENTION, ItemIntention.GRASP_ME);

		List<Vector2D> fakeList = new LinkedList<Vector2D>();
		List<BorderPoint> fakeList1 = new LinkedList<BorderPoint>();
		Region fakeRegion = new Region(fakeList, fakeList1);

		item.setAttribute(PropertyName.SURROUNDING, fakeRegion);

		List<ViewPoint> fakeList3 = new LinkedList<ViewPoint>();

		ViewPoints vps = new ViewPoints(fakeList3);
		item.setAttribute(PropertyName.VIEW_POINTS, vps);

		item.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
				.getMapAlgorithms().getBestViewPoint(vps.getPoints()));

		getManipulator().getItemMemory().addItemToQueue(item);

		new CalibrationGUI(getManipulator(), getManipulator().getItemMemory());

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}
	}
}
