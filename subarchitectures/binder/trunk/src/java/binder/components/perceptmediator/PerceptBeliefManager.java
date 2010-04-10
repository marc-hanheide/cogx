//package binder.components.perceptmediator;
//
//import java.util.HashMap;
//import java.util.LinkedList;
//import java.util.List;
//
//import binder.autogen.perceptmanagement.PerceptBeliefMaps;
//import cast.CASTException;
//import cast.architecture.ManagedComponent;
//import cast.cdl.WorkingMemoryAddress;
//import cast.core.CASTData;
//
//
//public class PerceptBeliefManager2 {
//	protected PerceptBeliefMaps pbm = null;
//	protected ManagedComponent component;
//	protected WorkingMemoryAddress wma;
//	protected static PerceptBeliefManager singleton = null;
//
//	public PerceptBeliefManager(ManagedComponent c) {
//		component = c;
//	}
//
//	public static PerceptBeliefManager getManager(ManagedComponent c) {
//		if (singleton==null) {
//			singleton = new PerceptBeliefManager(c);
//		}
//		return singleton;
//	}
//	
//	public static PerceptBeliefMaps readMaps(ManagedComponent c) throws CASTException {
//		getManager(c).read();
//		return getManager(c).pbm;
//	}
//	
//	public static void commitMaps(ManagedComponent c, PerceptBeliefMaps pbm) throws CASTException {
//		getManager(c).write(pbm);
//	}
//	
//	public void write(PerceptBeliefMaps pbm) throws CASTException {
//		if (wma==null) 
//			throw (new IllegalStateException("called write without prior accessing the map using read"));
//		this.pbm = pbm;
//		component.overwriteWorkingMemory(wma, pbm);
//	}
//	
//	public PerceptBeliefMaps read() throws CASTException {
//		List<CASTData<PerceptBeliefMaps>> result = new LinkedList<CASTData<PerceptBeliefMaps>>();
//
//		if (wma == null) {
//			component.getMemoryEntriesWithData(PerceptBeliefMaps.class, result, 1);
//
//			if (result.size()>0) {
//				String id=result.get(0).getID();
//				wma = new WorkingMemoryAddress(id, component.getSubarchitectureID());
//				pbm=result.get(0).getData();
//			}
//			else {
//				wma = new WorkingMemoryAddress(component.newDataID(), component.getSubarchitectureID());
//				pbm = new PerceptBeliefMaps(new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>());
//				component.addToWorkingMemory(wma, pbm);
//			}
//		}
//		else {
//			pbm=component.getMemoryEntry(wma, PerceptBeliefMaps.class);
//		}
//		return pbm;
//	}
//	
//}
