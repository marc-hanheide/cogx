/**
 * 
 */
package motivation.generators;

import motivation.factories.MotiveFactory;
import motivation.slice.Motive;
import motivation.slice.TestSource;
import NavData.FNode;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class TestGenerator extends Generator {
	@Override
	protected void runComponent() {
		println("Look out world, here I come...");

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(TestSource.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				Motive newMotive = MotiveFactory.createMotive(_wmc.address);
				// submit it to the working memory
				try {
					add(newMotive);
				} catch (AlreadyExistsOnWMException e) {
					log("shouldn't happen... the motive already existed...");
					e.printStackTrace();
				}
			}
		});

		String id=newDataID();
		try {
			// should also create a new Motive...
			log("adding new test source to WM");
			addToWorkingMemory(id, new TestSource());

			for (int i=0; i<10; i++) {
				log("updating test source in WM");
				overwriteWorkingMemory(id, new TestSource());
				Thread.sleep(2000);
			}

			log("delete test source in WM");
			deleteFromWorkingMemory(id);
		}
		catch(PermissionException e) {
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ConsistencyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}


	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.generators.Generator#checkMotive(cast.cdl.WorkingMemoryAddress
	 * , cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean updateMotive(WorkingMemoryAddress motiveAddress,
			WorkingMemoryAddress srcAddress) {
		try {
			debug("get motive from WM");
			Motive motive = getMemoryEntry(motiveAddress.id, Motive.class);
			debug("get source");
			TestSource source = getMemoryEntry(
					srcAddress.id, TestSource.class);
			// generate some fake stuff here...
			motive.goal = source.name;
			debug("before overwrite");
			overwrite(motiveAddress, motive);
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (ConsistencyException e) {
			e.printStackTrace();
		} catch (PermissionException e) {
			e.printStackTrace();
		}
		return false;
	}

}
