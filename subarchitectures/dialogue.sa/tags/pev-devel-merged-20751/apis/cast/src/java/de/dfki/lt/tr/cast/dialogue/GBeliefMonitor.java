package de.dfki.lt.tr.cast.dialogue;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.Date;
import java.util.Map;

import com.ibm.icu.text.SimpleDateFormat;

import autogen.Planner.POPlan;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import eu.cogx.beliefs.slice.GroundedBelief;

public class GBeliefMonitor extends ManagedComponent {
	
	private GBeliefMemory gbmemory = new GBeliefMemory();
	private boolean useUniqueFiles = false;
	
	protected void configure(Map<String, String> args) {
		super.configure(args);
		if (args.containsKey("--useUniqueFiles")) {
				
			if (args.get("--useUniqueFiles").equals("true")) {
				useUniqueFiles = true;
			}
		}
	}
	
	protected void start() {
		// add CFs

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(POPlan.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processAddedPOPlan(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(POPlan.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processOverwrittenPOPlan(_wmc);
			}
		});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processAddedGroundedBelief(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.OVERWRITE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processOverwrittenGroundedBelief(_wmc);
			}
		});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(GroundedBelief.class, WorkingMemoryOperation.DELETE), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
				processDeletedGroundedBelief(_wmc);
			}
		});
	}
	
	private void processAddedPOPlan(WorkingMemoryChange _wmc) {
		POPlan _newPOPlan;
		try {
			_newPOPlan = getMemoryEntry(_wmc.address, POPlan.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		
		if (!_newPOPlan.status.name().equals("RUNNING")) {
			writeToFile();
		}
	}
	
	private void processOverwrittenPOPlan(WorkingMemoryChange _wmc) {
		POPlan _oldPOPlan;
		try {
			_oldPOPlan = getMemoryEntry(_wmc.address, POPlan.class);
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		if (!_oldPOPlan.status.name().equals("RUNNING")) {
			writeToFile();
		}
	}
	
	private void processAddedGroundedBelief(WorkingMemoryChange _wmc) {
		GroundedBelief _newGroundedBelief;
		try {
			_newGroundedBelief = getMemoryEntry(_wmc.address, GroundedBelief.class);
			
			gbmemory.addGBelief(_wmc.address, getCASTTime(), _newGroundedBelief);
			
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		//log("Received new GroundedBelief");
	}
	
	private void processOverwrittenGroundedBelief(WorkingMemoryChange _wmc) {
		GroundedBelief _oldGroundedBelief;
		try {
			_oldGroundedBelief = getMemoryEntry(_wmc.address, GroundedBelief.class);
			
			gbmemory.addGBelief(_wmc.address, getCASTTime(), _oldGroundedBelief);
			
		} catch (DoesNotExistOnWMException e) {
			logException(e);
			return;
		} catch (UnknownSubarchitectureException e) {
			logException(e);
			return;
		}
		//log("Received overwritten GroundedBelief");
	}
	
	private void processDeletedGroundedBelief(WorkingMemoryChange _wmc) {
		
		gbmemory.addGBelief(_wmc.address, getCASTTime(), null);
		//log("Received deleted GroundedBelief");
	}
	
	private void writeToFile() {
		log("Writing GBeliefHistory ...");
		
		File file;
		if (useUniqueFiles) {
		    SimpleDateFormat sdf = new SimpleDateFormat("dd_MM_yy_hh_mm");
			file = new File("GBeliefHistory_" + sdf.format(new Date()));
		} else {
		    file = new File("GBeliefHistory");
		}
		
		try {
			FileOutputStream f = new FileOutputStream(file);
			ObjectOutputStream s = new ObjectOutputStream(f);
			s.writeObject(gbmemory);
			s.flush();
			s.close();
			f.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		 catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void readFromFile() {
		
		File file = new File("GBeliefHistory");
		try {
			FileInputStream f = new FileInputStream(file);
			ObjectInputStream s = new ObjectInputStream(f);
			gbmemory = (GBeliefMemory) s.readObject();
			s.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	     catch (ClassNotFoundException e) {
		// TODO Auto-generated catch block
		  e.printStackTrace();
	    }
		 catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	protected void runComponent() {
		log("GBeliefMonitor running...");
	}
}
