package manipulation.core.bham.simulationConnector;

import golem.tinyice.RigidBodyPrx;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.itemMemory.Item;

import org.apache.log4j.Logger;

/**
 * Runnable to continuously update the item positions
 * 
 * @author ttoenige
 * 
 */
public class UpdateItemPositionRunnable implements Runnable {
	private Logger logger = Logger.getLogger(this.getClass());
	
	
	private BhamSimulationConnector simCon;
	private Manipulator manipulator;

	/**
	 * constructor for the item update runnable
	 * 
	 * @param simCon
	 *            current virtual scene connector
	 * @param manipulator
	 *            current manipulator
	 */
	public UpdateItemPositionRunnable(BhamSimulationConnector simCon,
			Manipulator manipulator) {
		this.manipulator = manipulator;
		this.simCon = simCon;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while (!Thread.interrupted()) {
			for (RigidBodyPrx obstacle : simCon.getObstacles()) {
				try {
					simCon.getTinyInterface().releaseActor(obstacle);
				} catch (Exception e) {
					logger.error(e);
				}
			}
			simCon.getObstacles().clear();

			for (RigidBodyPrx table : simCon.getTables()) {
				try {
					simCon.getTinyInterface().releaseActor(table);
				} catch (Exception e) {
					logger.error(e);
				}
			}

			simCon.getTables().clear();

			try {
				for (Item item : manipulator.getItemMemory().getItemList()) {
					simCon.getObstacles().add(simCon.updateObstacle(item));

					simCon.getTables().add(simCon.generateTable(item));
				}
			} catch (InternalMemoryException e) {
				// logger.warn(e);
			} catch (Exception e) {
				logger.error(e);
			}

			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				return;
			}
		}
	}
}
