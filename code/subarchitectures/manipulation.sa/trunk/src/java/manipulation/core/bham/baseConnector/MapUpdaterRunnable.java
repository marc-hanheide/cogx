package manipulation.core.bham.baseConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;

import org.apache.log4j.Logger;

/**
 * Runnable to update the map with a constant interval
 * 
 * @author ttoenige
 * 
 */
public class MapUpdaterRunnable implements Runnable {
	private Manipulator manipulator;

	private Logger logger = Logger.getLogger(this.getClass());
	
	/**
	 * constructor for the runnable to update the map with a constant interval
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public MapUpdaterRunnable(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void run() {

		while (!Thread.interrupted()) {
			try {
				Thread.sleep(5000);

				manipulator.getMapConnector().updateMap();

//				Item currentItem = manipulator.getItemMemory()
//						.getFirstGraspItem();
//
//				List<ViewPoint> vps = manipulator.getMapAlgorithms()
//						.generatePotentialViewPoints(currentItem);
//
//				ViewPoint bestVP = manipulator.getMapAlgorithms()
//						.getBestViewPoint(vps);
//
//				manipulator.getItemMemory().updateViewPoints(currentItem,
//						new ViewPoints(vps), bestVP);

			} catch (ExternalMemoryException e) {
				logger.error(e);
			} catch (InterruptedException e) {
				return;
//			} catch (ItemException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			} catch (MathException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			} catch (InternalMemoryException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
			}

		}
	}
}
