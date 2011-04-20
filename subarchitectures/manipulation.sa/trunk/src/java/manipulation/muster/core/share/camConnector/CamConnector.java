package manipulation.muster.core.share.camConnector;

import manipulation.muster.core.share.exceptions.ExternalMemoryException;
import manipulation.muster.itemMemory.Item;

/**
 * represents a connection to the camera and the visual system
 * 
 * @author ttoenige
 * 
 */
public interface CamConnector {

	/**
	 * starts tracking of an item
	 * 
	 * @throws ExternalMemoryException
	 */
	public void startTracking() throws ExternalMemoryException;

	/**
	 * stops tracking of an item
	 * 
	 * @throws ExternalMemoryException
	 */
	public void stopTracking() throws ExternalMemoryException;

	/**
	 * recognize an item and starts to track it
	 * 
	 * @param item
	 *            item to recognize and to track
	 * @return <code>true</code> if item is recognized, <code>false</code> if
	 *         item is not recognized
	 * @throws ExternalMemoryException
	 */
	public boolean recognizeTrackItem(Item item) throws ExternalMemoryException;

	/**
	 * resets the tracker
	 * 
	 * @throws ExternalMemoryException
	 */
	public void resetTracker() throws ExternalMemoryException;

}
