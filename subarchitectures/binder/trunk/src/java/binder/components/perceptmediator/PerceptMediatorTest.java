/**
 * 
 */
package binder.components.perceptmediator;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProperties.ConnectivityPathProperty;
import SpatialProperties.GatewayPlaceProperty;
import cast.CASTException;
import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public class PerceptMediatorTest extends ManagedComponent {

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		try {
			String id=newDataID();
			addToWorkingMemory(id, new Place(0, PlaceStatus.PLACEHOLDER));
			Thread.sleep(100);
			addToWorkingMemory(newDataID(), new Place(1, PlaceStatus.PLACEHOLDER));
			Thread.sleep(100);
			addToWorkingMemory(newDataID(), new Place(2, PlaceStatus.TRUEPLACE));
			Thread.sleep(200);			
			overwriteWorkingMemory(id, new Place(0, PlaceStatus.TRUEPLACE));
			Thread.sleep(200);			
			deleteFromWorkingMemory(id);
			
			Thread.sleep(200);				
			String connid=newDataID();
			addToWorkingMemory(connid, new ConnectivityPathProperty(3, 2, null, null, true));

			Thread.sleep(200);				
			addToWorkingMemory(newDataID(), new GatewayPlaceProperty(3, null, null, true));

			
			Thread.sleep(500);
			addToWorkingMemory(newDataID(), new Place(3, PlaceStatus.TRUEPLACE));

			
			
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

}
