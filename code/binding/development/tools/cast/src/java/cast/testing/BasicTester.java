/**
 * 
 */
package cast.testing;

import java.util.HashMap;
import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ConsistencyException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
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
import cast.core.data.CASTData;

/**
 * @author nah
 * 
 */
public class BasicTester extends AbstractTester {

	/**
	 * 
	 * Test: Writes out a struct, then reads it back in from the same location
	 * and verifies its contents.
	 * 
	 * @author nah
	 * 
	 */
	class SingleComponentReadWriteTest extends AbstractTest {
		@Override
		protected void startTest() {

			try {
				String id = newDataID();

				CASTTestStruct wrote = createTestStruct(id);

				addToWorkingMemory(id, wrote, OperationMode.BLOCKING);

				CASTTestStruct read = (CASTTestStruct) getWorkingMemoryEntry(id)
						.getData();

				if (!CASTUtils.equals(wrote, read)) {
					testComplete(false);
				}

				// add a few more
				addToWorkingMemory(newDataID(), wrote, OperationMode.BLOCKING);
				addToWorkingMemory(newDataID(), wrote, OperationMode.BLOCKING);
				addToWorkingMemory(newDataID(), wrote, OperationMode.BLOCKING);
				addToWorkingMemory(newDataID(), wrote, OperationMode.BLOCKING);

				CASTData<CASTTestStruct>[] entries = getWorkingMemoryEntries(CASTTestStruct.class);

				if (entries.length != 5) {
					testComplete(false);
				}

				for (CASTData<CASTTestStruct> data : entries) {
					if (!CASTUtils.equals(wrote, data.getData())) {
						testComplete(false);
					}
				}

				entries = getWorkingMemoryEntries(getSubarchitectureID(),
						CASTTestStruct.class, 4);
				if (entries.length != 4) {
					testComplete(false);
				}

				testComplete(true);

			}
			catch (Exception e) {
				testComplete(false);
			}

		}

	}

	private CASTTestStruct createTestStruct(String _id) {
		return createTestStruct(_id, getSubarchitectureID(), 0);
	}

	private CASTTestStruct createTestStruct(String _id, String _subarch) {
		return createTestStruct(_id, _subarch, 0);
	}

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
	 * Test: Writes out a struct, waits for another component to copy it to a
	 * new location, then reads in the copy and compares to initial data.
	 * 
	 * @author nah
	 * 
	 */
	class TwoComponentReadWriteTest extends AbstractTest
			implements
				WorkingMemoryChangeReceiver {

		private CASTTestStruct m_wrote;

		@Override
		protected void startTest() {

			try {

				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD), this);

				// wait before starting... allows other components to get their
				// filters up
				sleepProcess(1000);

				String id = newDataID();

				m_wrote = createTestStruct(id);

				addToWorkingMemory(id, m_wrote, OperationMode.BLOCKING);

			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			// only if it's not my change
			if (!_wmc.m_src.equals(getProcessIdentifier())) {
				try {

					// println("seen change again");
					CASTTestStruct read = (CASTTestStruct) getWorkingMemoryEntry(
							_wmc.m_address.m_id).getData();

					// println(read.m_count);
					// println(CASTUtils.toString(read.m_change));
					// println(m_wrote.m_count);
					// println(CASTUtils.toString(m_wrote.m_change));
					//					
					// println(read.equals(m_wrote));
					//					
					testComplete(CASTUtils.equals(read, m_wrote));

				}
				catch (Exception e) {
					e.printStackTrace();
					testComplete(false);
				}
			}
		}
	}

	/**
	 * Handles the other end of the interaction with TwoComponentReadWriteTest
	 * 
	 * @author nah
	 * 
	 */
	class Copier extends AbstractTest implements WorkingMemoryChangeReceiver {
		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD), this);
			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			try {
				// read struct in
				CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
						_wmc.m_address.m_id).getData();

				// then just write it out again
				addToWorkingMemory(newDataID(), cts);
				removeChangeFilter(this);

				testComplete(true);

			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}
	}

	/**
	 * Waits for a change then overwrites the target with its own struct.
	 * 
	 * @author nah
	 * 
	 */
	class Replacer extends AbstractTest implements WorkingMemoryChangeReceiver {
		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD), this);
			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			try {

				// println("seen change");

				String id = _wmc.m_address.m_id;
				// get it
				getWorkingMemoryEntry(_wmc.m_address);

				// then just write it out again
				overwriteWorkingMemory(id, createTestStruct(id));

				// println("and overwritten");

			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}
	}

	/**
	 * Waits for a delete then readds the target with its own struct.
	 * 
	 * @author nah
	 * 
	 */
	class DoesNotExistTest extends AbstractTest {

		@Override
		protected void startTest() {
			try {
				getWorkingMemoryEntry(newDataID(), m_targetSubarch);
				testComplete(false);
			}
			catch (DoesNotExistOnWMException e) {
				// this is the exception we want
				e.printStackTrace();
				testComplete(true);
			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}
	}

	/**
	 * Waits for a delete then readds the target with its own struct.
	 * 
	 * @author nah
	 * 
	 */
	class ReAdder extends AbstractTest implements WorkingMemoryChangeReceiver {

		private CASTTestStruct m_wrote;

		@Override
		protected void startTest() {

			try {

				sleepProcess(1000);
				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD), this);
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(
								CASTTestStruct.class,
								WorkingMemoryOperation.OVERWRITE), this);
				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.DELETE),
						this);

				String id = newDataID();
				m_wrote = createTestStruct(id);
				addToWorkingMemory(id, m_wrote);

			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			try {
				if (++m_wrote.m_count == 100) {
					testComplete(true);
					return;
				}

				if (m_wrote.m_count % 5 == 2) {
					log("adding on count: " + m_wrote.m_count);
					addToWorkingMemory(_wmc.m_address, m_wrote);
					lockEntry(_wmc.m_address, WorkingMemoryPermissions.LOCKED_O);
				}
				else if (m_wrote.m_count % 5 == 1) {
					log("nothing on count: " + m_wrote.m_count);
				}
				else {
					log("overwrite on count: " + m_wrote.m_count);
					overwriteWorkingMemory(_wmc.m_address, m_wrote);
				}

			}
			catch (Exception e) {
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
	class Writer extends AbstractTest {

		private final int m_count;

		public Writer(int _count) {
			m_count = _count;
		}

		@Override
		protected void startTest() {

			// wait before starting... allows other components to get their
			// filters up
			try {
				Thread.sleep(1000);
			}
			catch (InterruptedException e1) {
				e1.printStackTrace();
			}

			for (int i = 0; i < m_count; i++) {
				String id = newDataID();
				CASTTestStruct cts = createTestStruct(id, m_targetSubarch, i);
				try {
					addToWorkingMemory(id, cts);
				}
				catch (SubarchitectureProcessException e) {
					e.printStackTrace();
					testComplete(false);
				}
			}

			// sleep a little just in case something goes goes elsewhere
			sleepProcess(2000);
			testComplete(true);
		}

	}

	/**
	 * Subarchitecture used as target for testing operations. Defaults to own
	 * subarch.
	 */
	private String m_targetSubarch;

	/**
	 * Tests combinations of receivers
	 * 
	 */
	class Receiver extends AbstractTest {

		@Override
		protected void startTest() {

			String id = newDataID();

			// add all combinations of receivers
			try {

				log("# filters: " + getFilterCount());

				// wildcard global
				addChangeFilter(
						ChangeFilterFactory
								.createOperationFilter(WorkingMemoryOperation.WILDCARD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: FilterRestriction.LOCAL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// wildcard local
				addChangeFilter(ChangeFilterFactory.createOperationFilter(
						WorkingMemoryOperation.WILDCARD,
						FilterRestriction.ALL_SA),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: FilterRestriction.ALL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// typed local
				addChangeFilter(ChangeFilterFactory
						.createLocalTypeFilter(CASTTestStruct.class),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: type FilterRestriction.LOCAL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// typed global
				addChangeFilter(ChangeFilterFactory
						.createGlobalTypeFilter(CASTTestStruct.class),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: type FilterRestriction.ALL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// untyped add local
				addChangeFilter(ChangeFilterFactory
						.createOperationFilter(WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: add FilterRestriction.LOCAL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// untyped add global
				addChangeFilter(ChangeFilterFactory.createOperationFilter(
						WorkingMemoryOperation.ADD, FilterRestriction.ALL_SA),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: add FilterRestriction.ALL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// typed add local
				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: type add FilterRestriction.LOCAL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// typed add global
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: type add FilterRestriction.ALL_SA");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// full version
				addChangeFilter(ChangeFilterFactory.createIDFilter(id),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: explicit wildcards");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// full version
				addChangeFilter(ChangeFilterFactory
						.createAddressFilter(new WorkingMemoryAddress(id,
								getSubarchitectureID())),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: explicit wildcards");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

				// full version
				addChangeFilter(ChangeFilterFactory
						.createSourceFilter(getProcessIdentifier()),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								log("addChangeFilter: explicit wildcards");
								try {
									removeChangeFilter(this);
								}
								catch (SubarchitectureProcessException e) {
									e.printStackTrace();
								}
							}
						});

				log("# filters: " + getFilterCount());

			}
			catch (SubarchitectureProcessException e1) {
				e1.printStackTrace();
				testComplete(false);
			}

			// // sanity check
			// if (24 != getFilterCount()) {
			// testComplete(false);
			// }

			CASTTestStruct cts = createTestStruct(id, m_targetSubarch);
			try {
				// unlock to allow changes to be received
				unlockProcess();

				addToWorkingMemory(cts.m_change.m_address, cts);

			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

			// wait a while
			sleepProcess(1000);
			log("remaining filters: " + getFilterCount());
			if (getFilterCount() == 0) {
				testComplete(true);
			}
			else {
				testComplete(false);
			}

			// relock to allow test to continue
			lockProcess();

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
	class Overwriter extends AbstractTest {

		private final int m_count;

		private final boolean m_safe;

		public Overwriter(int _count, boolean _safe) {
			m_count = _count;
			m_safe = _safe;
		}

		@Override
		protected void startTest() {

			// wait before starting... allows other components to get their
			// filters up
			try {
				Thread.sleep(1000);
			}
			catch (InterruptedException e1) {
				e1.printStackTrace();
			}

			String id = newDataID();

			try {
				addToWorkingMemory(id, m_targetSubarch, createTestStruct(id,
						m_targetSubarch, 0), OperationMode.BLOCKING);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				println(e.getLocalizedMessage());
				testComplete(false);
				return;
			}

			for (int i = 0; i < m_count; i++) {
				// sleep to allow others to interfere
				sleepProcess(500);
				try {

					// if safe, then read in before overwrite
					if (m_safe) {
						getWorkingMemoryEntry(id, m_targetSubarch);
					}

					try {
						overwriteWorkingMemory(id, m_targetSubarch,
								createTestStruct(id, m_targetSubarch, i),
								OperationMode.BLOCKING);

					}
					catch (ConsistencyException e) {
						if (m_safe) {
							log("consistency check failed");
							log(e.getLocalizedMessage());
							e.printStackTrace();
							testComplete(false);
							return;
						}
					}

				}
				catch (SubarchitectureProcessException e) {
					e.printStackTrace();
					println(e.getLocalizedMessage());
					testComplete(false);
					return;
				}
			}
			testComplete(true);
		}

	}

	/**
	 * 
	 * For every source of test structs, this component needs to receive m_count
	 * structs in order.
	 * 
	 * @author nah
	 * 
	 */
	class Counter extends AbstractTest implements WorkingMemoryChangeReceiver {

		private final int m_expecting;

		private HashMap<String, Integer> m_counts;

		public Counter(int _count) {
			m_expecting = _count;
			m_counts = new HashMap<String, Integer>();
		}

		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD), this);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			// read in the change
			try {
				CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(
						_wmc.m_address).getData();

				String src = cts.m_change.m_src;
				int count = cts.m_count;

				// if we've seen changes from here before
				if (m_counts.containsKey(src)) {

					// get the existing count
					int currentCount = m_counts.get(src);

					// if this isn't true then we're out of step
					if (currentCount != (count - 1)) {
						System.out.println("failed on order");
						testComplete(false);
					}
					// if this is true then we've received all the ones we
					// need
					else if (count == (m_expecting - 1)) {
						m_counts.remove(src);
					}
					// else just store the count
					else {

						// if(count % 10 == 0) {
						// log(src + " " + count);
						// }

						m_counts.put(src, count);
					}

				}
				else {
					// the first count should be 0
					if (count != 0) {
						System.out.println("failed on first");
						testComplete(false);
					}
					m_counts.put(src, count);
				}

			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}

			// if we're empty, then we're complete!
			if (m_counts.isEmpty()) {
				// just in case
				sleepProcess(10000);
				testComplete(true);
			}

		}

	}

	/**
	 * Waits for a change then overwrites the target with its own struct.
	 * 
	 * @author nah
	 * 
	 */
	class Deleter extends AbstractTest implements WorkingMemoryChangeReceiver {

		private final int m_modulo;

		public Deleter(int _modulo) {
			m_modulo = _modulo;
		}

		@Override
		protected void startTest() {

			try {
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
						CASTTestStruct.class, WorkingMemoryOperation.ADD), this);
				addChangeFilter(
						ChangeFilterFactory.createGlobalTypeFilter(
								CASTTestStruct.class,
								WorkingMemoryOperation.OVERWRITE), this);
			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			try {

				println("seen change");

				WorkingMemoryAddress wma = _wmc.m_address;

				// get it -- actually not necessary - bug #52
				CASTTestStruct cts = (CASTTestStruct) getWorkingMemoryEntry(wma)
						.getData();

				// then delete it
				if (cts.m_count % m_modulo == 0) {
					deleteFromWorkingMemory(wma, OperationMode.BLOCKING);
					log("deleted on count: " + cts.m_count + ": "
							+ CASTUtils.toString(wma));
				}
				else {
					log("left on count: " + cts.m_count + ": "
							+ CASTUtils.toString(wma));
				}

				// check if it exists now
				// if (existsOnWorkingMemory(wma)) {
				// testComplete(false);
				// removeChangeFilter(this);
				// } else {

				// }

				// println("and overwritten");

			}
			catch (Exception e) {
				e.printStackTrace();
				testComplete(false);
			}

		}
	}

	/**
	 * @param _id
	 */
	public BasicTester(String _id) {
		super(_id);
	}

	@Override
	public void configure(Properties _config) {
		try {
			registerTest("single-write", new SingleComponentReadWriteTest());
			registerTest("read-write", new TwoComponentReadWriteTest());
			registerTest("copy", new Copier());
			registerTest("write-10", new Writer(10));
			registerTest("count-10", new Counter(10));
			registerTest("write-100", new Writer(100));
			registerTest("count-100", new Counter(100));
			registerTest("write-1000", new Writer(1000));
			registerTest("count-1000", new Counter(1000));
			registerTest("overwrite", new Overwriter(10, true));
			registerTest("unsafe-overwrite", new Overwriter(10, false));
			registerTest("replace", new Replacer());
			registerTest("delete", new Deleter(1));
			registerTest("delete-5", new Deleter(5));
			registerTest("receive", new Receiver());
			registerTest("readd", new ReAdder());
			registerTest("dne", new DoesNotExistTest());
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
