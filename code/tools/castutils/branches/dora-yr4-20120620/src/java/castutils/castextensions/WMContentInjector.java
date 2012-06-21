/**
 * 
 */
package castutils.castextensions;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.production.PlanVerbalizationRequest;

/**
 * @author marc
 * 
 */
public class WMContentInjector extends CASTHelper {

	/**
	 * @param c
	 */
	public WMContentInjector(ManagedComponent c) {
		super(c);
	}

	public void inject(String xmlStr) {
		component.log("WM Entry to inject:\n" + xmlStr);
		WMInjection wmi = null;
                try {
			wmi = IceXMLSerializer.fromXMLString(xmlStr,
				WMInjection.class);
		} catch (Exception e) {
			component.logException(e);
			return;
		}
		
		// if the id is empty, generate a new one
		if (wmi.adr.id.length() == 0)
			wmi.adr.id = component.newDataID();
		// is SA is empty, use the SA this component is running in
		if (wmi.adr.subarchitecture.length() == 0)
			wmi.adr.subarchitecture = component.getSubarchitectureID();
		component.log("WM Entry to inject:\n" + IceXMLSerializer.toXMLString(wmi));
		try {
			switch (wmi.op) {
			case ADD:
				component.addToWorkingMemory(wmi.adr, wmi.content);
				break;
			case DELETE:
				component.deleteFromWorkingMemory(wmi.adr);
				break;
			case OVERWRITE:
				component.overwriteWorkingMemory(wmi.adr, wmi.content);
				break;
			default:
			}
		} catch (CASTException e) {
			component.logException(e);
		}
	}
	
	public static void main(String[] args) {
		
		WMInjection wmi = new WMInjection();
		wmi.adr=new WorkingMemoryAddress("id","sa");
		wmi.op=WorkingMemoryOperation.ADD;
		wmi.content=new PlanVerbalizationRequest(4);
		System.out.println(IceXMLSerializer.toXMLString(wmi));
		
	}

}
