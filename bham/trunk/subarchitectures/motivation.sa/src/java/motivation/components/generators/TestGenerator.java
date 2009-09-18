/**
 * 
 */
package motivation.components.generators;

import motivation.factories.MotiveFactory;
import motivation.slice.Motive;
import motivation.slice.TestSource;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class TestGenerator extends AbstractMotiveGenerator {
	
	
	@Override
	protected void runComponent() {
		println("Look out world, here I come...");

		String id=newDataID();
		try {
			// should also create a new Motive...
			log("adding new test source to WM");
			addToWorkingMemory(id, new TestSource());

			for (int i=0; i<10; i++) {
				log("updating test source in WM");
				lockEntry(id, WorkingMemoryPermissions.LOCKEDO);
				TestSource ts = getMemoryEntry(id,TestSource.class);
				overwriteWorkingMemory(id, ts);
				unlockEntry(id);
				Thread.sleep(1000);
				addToWorkingMemory(newDataID(), new TestSource());
				Thread.sleep(1000);
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
	protected boolean checkMotive(Motive motive) {
		try {
			getMemoryEntry(motive.referenceEntry, TestSource.class);
			// generate some fake stuff here...
			write(motive);
			return true;
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}
		return false;
	}


	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		// TODO Auto-generated method stub
		super.start();
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(TestSource.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				Motive newMotive = MotiveFactory.createTestMotive(_wmc.address);
				checkMotive(newMotive);
			}
		});

	}


	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		// TODO Auto-generated method stub
		super.stop();
	}

}
