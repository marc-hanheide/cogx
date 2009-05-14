package org.cognitivesystems.comsys.general.testers;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFPacking;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;
import cast.testing.AbstractTester;
import java.util.Vector;

public class AbstractComsysTester extends AbstractTester {

	/**
	 * @param _id
	 */
	public AbstractComsysTester(String _id) {
		super(_id);
	}
	
	abstract public class AbstractComsysTest extends AbstractTest  {

		protected Vector<String> parses;

		protected void initParseTest (String utterance, Vector<String> parses) {

			this.parses = parses;

			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedPackedLF(_wmc);
							}
						}); 

				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedPackedLF(_wmc);
							}
						});
				log("Filters OK");

				PhonString phonString = new PhonString();
				phonString.wordSequence = utterance;
				String id = newDataID();
				phonString.id = id;
				log("Inserting the phonstring 1 into the working memory");
				addToWorkingMemory(id, phonString);

				sleepProcess(800);

			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}
		}


		/**
		 * @param _wmc
		 * @param i
		 */
		private void addedPackedLF(WorkingMemoryChange _wmc) {
			try {

				log("got added packedlf");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData plfWM = new CASTData(id,
						getWorkingMemoryEntry(id));
				PackedLFs plf = (PackedLFs) plfWM.getData();
				checkPLF(plf);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
		} 
		
		public void checkPLF(PackedLFs plf) {}

	}
}