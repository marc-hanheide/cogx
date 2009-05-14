/**
 * 
 */
package cast.testing;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PermissionException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.FilterRestriction;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.testing.CASTTestStruct;
import cast.core.CASTException;
import cast.core.CASTUtils;

/**
 * @author nah
 * 
 */
public class LockTester extends AbstractTester {

	private CASTTestStruct createTestStruct(String _id, String _subarch,
			int _count) {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(_id, _subarch);

		CASTTestStruct wrote = new CASTTestStruct(_count,
				new WorkingMemoryChange(WorkingMemoryOperation.ADD,
						getProcessIdentifier(), wma, CASTUtils
								.typeName(CASTTestStruct.class)));
		return wrote;
	}
	/**
	 * 
	 * Create m_count numbers of structs as fast as it can. the m_count field of
	 * the struct will be incremented accordingly.
	 * 
	 * @author nah
	 * 
	 */
	class Locker extends AbstractTest {

		final private WorkingMemoryPermissions m_permissions;

		public Locker(WorkingMemoryPermissions _permissions) {
			m_permissions = _permissions;
		}

		@Override
		protected void startTest() {
			try {
				sleepProcess(2000);
				String id = newDataID();
				CASTTestStruct cts = createTestStruct(id, m_targetSubarch, 0);
				addToWorkingMemory(id, m_targetSubarch, cts);
				lockEntry(id, m_targetSubarch, m_permissions);
				sleepProcess(3000);
				unlockEntry(id, m_targetSubarch);
				testComplete(true);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}
		}

	}
	/**
	 * 
	 * Create m_count numbers of structs as fast as it can. the m_count field of
	 * the struct will be incremented accordingly.
	 * 
	 * @author nah
	 * 
	 */
	class LockingWriter extends AbstractTest
			implements
				WorkingMemoryChangeReceiver {

		private final int m_count;

		public LockingWriter(int _count) {
			m_count = _count;
		}

		@Override
		protected void startTest() {
			sleepProcess(2000);

			String id = newDataID();

			try {
				addChangeFilter(ChangeFilterFactory.createAddressFilter(id,
						getSubarchitectureID(),
						WorkingMemoryOperation.OVERWRITE), this);

				// wait before starting... allows other components to get their
				// filters up
				try {
					Thread.sleep(1000);
				}
				catch (InterruptedException e1) {
					e1.printStackTrace();
				}

				CASTTestStruct cts = createTestStruct(id, m_targetSubarch, 0);
				addToWorkingMemory(id, m_targetSubarch, cts);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			try {
				println(CASTUtils.toString(getPermissions(_wmc.m_address)));

				println("reading: " + CASTUtils.toString(_wmc.m_address));
				CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
						_wmc.m_address).getData();
				println("read: " + CASTUtils.toString(_wmc.m_address) + " "
						+ cts.m_count);

				if (cts.m_count == m_count) {
					println("deleting: " + CASTUtils.toString(_wmc.m_address));
					deleteFromWorkingMemory(_wmc.m_address,
							OperationMode.BLOCKING);
					println("deleted: " + CASTUtils.toString(_wmc.m_address));

					removeChangeFilter(this);
					testComplete(true);
				}
				else {
					log("unlocking: " + CASTUtils.toString(_wmc.m_address));
					unlockEntry(_wmc.m_address);
					log("unlocked: " + CASTUtils.toString(_wmc.m_address));
				}

			}
			catch (SubarchitectureProcessException e) {
				println(e);
				try {
					removeChangeFilter(this);
				}
				catch (SubarchitectureProcessException e1) {
					println(e1);
				}
				testComplete(false);
			}
		}

	}

	/**
	 * Subarchitecture used as target for testing operations. Defaults to own
	 * subarch.
	 */
	private String m_targetSubarch;

	/**
	 * 
	 * Overwrites a single struct m_count times, with a configurable gap between
	 * the attempts.
	 * 
	 * @author nah
	 * 
	 */
	class LockingOverwriter extends AbstractTest
			implements
				WorkingMemoryChangeReceiver {

		private final int m_count;

		public LockingOverwriter(int _count) {
			m_count = _count;
		}

		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD,
						FilterRestriction.ALL_SA), this);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			// try {
			// println(CASTUtils.toString(getPermissions(_wmc.m_address)));
			// } catch (WMException e) {
			// e.printStackTrace();
			// }

			try {

				// lock the entry
				log("locking: " + CASTUtils.toString(_wmc.m_address));
				lockEntry(_wmc.m_address, WorkingMemoryPermissions.LOCKED_ODR);
				log("locked: " + CASTUtils.toString(_wmc.m_address));

				println("reading: " + CASTUtils.toString(_wmc.m_address));
				CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
						_wmc.m_address).getData();
				println("read: " + CASTUtils.toString(_wmc.m_address) + " "
						+ cts.m_count);

				for (int i = 0; i < m_count; i++) {
					cts.m_count++;
					overwriteWorkingMemory(_wmc.m_address, cts);
					sleepProcess(100);
				}

				log("unlocking: " + CASTUtils.toString(_wmc.m_address));
				unlockEntry(_wmc.m_address);
				log("unlocked: " + CASTUtils.toString(_wmc.m_address));

				testComplete(true);
			}
			catch (SubarchitectureProcessException e) {
				println(e);
				try {
					removeChangeFilter(this);
				}
				catch (SubarchitectureProcessException e1) {
					println(e1);
				}
				testComplete(false);
				return;
			}
		}

	}

	/**
	 * 
	 * Overwrites a single struct m_count times, with a configurable gap between
	 * the attempts.
	 * 
	 * @author nah
	 * 
	 */
	class TryLockingOverwriter extends AbstractTest
			implements
				WorkingMemoryChangeReceiver {

		private final int m_count;

		public TryLockingOverwriter(int _count) {
			m_count = _count;
		}

		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD,
						FilterRestriction.ALL_SA), this);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			try {

				// try locking the entry
				log("try locking: " + CASTUtils.toString(_wmc.m_address));
				while (!tryLockEntry(_wmc.m_address,
						WorkingMemoryPermissions.LOCKED_ODR)) {
					log("try locking: " + CASTUtils.toString(_wmc.m_address));
					sleepProcess(100);
				}
				log("locked: " + CASTUtils.toString(_wmc.m_address));

				println("reading: " + CASTUtils.toString(_wmc.m_address));
				CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
						_wmc.m_address).getData();
				println("read: " + CASTUtils.toString(_wmc.m_address) + " "
						+ cts.m_count);

				for (int i = 0; i < m_count; i++) {
					cts.m_count++;
					overwriteWorkingMemory(_wmc.m_address, cts);
					sleepProcess(100);
				}

				log("unlocking: " + CASTUtils.toString(_wmc.m_address));
				unlockEntry(_wmc.m_address);
				log("unlocked: " + CASTUtils.toString(_wmc.m_address));

				testComplete(true);

			}
			catch (SubarchitectureProcessException e) {
				println(e);
				try {
					removeChangeFilter(this);
				}
				catch (SubarchitectureProcessException e1) {
					println(e1);
				}
				testComplete(false);
				return;
			}
		}

	}

	/**
	 * 
	 * Overwrites a single struct m_count times, with a configurable gap between
	 * the attempts.
	 * 
	 * @author nah
	 * 
	 */
	class Sneaker extends AbstractTest implements WorkingMemoryChangeReceiver {

		private final WorkingMemoryOperation m_op;

		public Sneaker(WorkingMemoryOperation _op) {
			m_op = _op;
		}

		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD,
						FilterRestriction.ALL_SA), this);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			try {
				// need to wait a little while
				sleepProcess(500);

				if (m_op == WorkingMemoryOperation.OVERWRITE) {
					try {
						CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
								_wmc.m_address).getData();
						overwriteWorkingMemory(_wmc.m_address, cts);
						testComplete(false);
					}
					catch (PermissionException p) {
						testComplete(true);
					}
				}
				else if (m_op == WorkingMemoryOperation.DELETE) {

					try {
						CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
								_wmc.m_address).getData();
						overwriteWorkingMemory(_wmc.m_address, cts);
						testComplete(false);
					}
					catch (PermissionException p) {
						try {
							deleteFromWorkingMemory(_wmc.m_address);
							testComplete(false);
						}
						catch (PermissionException q) {
							testComplete(true);
						}
					}

				}
				else if (m_op == WorkingMemoryOperation.GET) {
					try {
						overwriteWorkingMemory(_wmc.m_address,
								createTestStruct(_wmc.m_address.m_id,
										_wmc.m_address.m_subarchitecture, 1));
						log("overwrite incorrectly allowed");
						testComplete(false);
					}
					catch (PermissionException p) {
						try {
							deleteFromWorkingMemory(_wmc.m_address);
							log("deletion incorrectly allowed");
							testComplete(false);
						}
						catch (PermissionException q) {
							if (isReadable(_wmc.m_address.m_id,
									_wmc.m_address.m_subarchitecture)) {
								log("read incorrectly allowed");
								testComplete(false);
							}
							else {
								testComplete(true);
							}
						}
					}

				}
				else {
					assert (true);
				}
			}
			catch (SubarchitectureProcessException e) {
				println(e);
				testComplete(false);
			}
		}
	}

	/**
	 * @param _id
	 */
	public LockTester(String _id) {
		super(_id);
	}

	@Override
	public void configure(Properties _config) {
		try {
			registerTest("overwrite-10", new LockingOverwriter(10));
			registerTest("try-overwrite-10", new TryLockingOverwriter(10));
			registerTest("write-10", new LockingWriter(10));
			registerTest("write-40", new LockingWriter(40));

			registerTest("lock-o",
					new Locker(WorkingMemoryPermissions.LOCKED_O));
			registerTest("lock-od", new Locker(
					WorkingMemoryPermissions.LOCKED_OD));
			registerTest("lock-odr", new Locker(
					WorkingMemoryPermissions.LOCKED_ODR));

			registerTest("sneak-o", new Sneaker(WorkingMemoryOperation.OVERWRITE));
			registerTest("sneak-d", new Sneaker(WorkingMemoryOperation.DELETE));
			registerTest("sneak-r", new Sneaker(WorkingMemoryOperation.GET));

		}
		catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

		// set the target subarchitecture for operations
		m_targetSubarch = _config.getProperty("--subarch",
				getSubarchitectureID());

	}

}
