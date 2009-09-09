/* * Comedian example code to demonstrate CAST functionality. Copyright
 * (C) 2006-2007 Nick Hawes This library is free software; you can
 * redistribute it and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software Foundation;
 * either version 2.1 of the License, or (at your option) any later
 * version. This library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. You should have
 * received a copy of the GNU Lesser General Public License along with
 * this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

package comedyarch;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.WMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import cast.core.CASTUtils;

import comedyarch.autogen.Joke;
import comedyarch.autogen.OneLiner;
import comedyarch.autogen.TwoLiner;

/**
 * @author nah
 */
public class FunnyMan extends ManagedComponent {

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {

		// create a new receiver object that just calls a member
		// function from this class
		WorkingMemoryChangeReceiver receiver = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				newTwoLinerAdded(_wmc);
			}
		};

		// add this receiver to listen for changes
		addChangeFilter(
		// listen for joke types that are
				// added that are local to this
				// subarchitecture
				ChangeFilterFactory.createLocalTypeFilter(TwoLiner.class,
						WorkingMemoryOperation.ADD), receiver);

		// examples to demonstrate matching on supertypes

//		addChangeFilter(
//				// listen for joke types that are
//				// added that are local to this
//				// subarchitecture
//				ChangeFilterFactory.createLocalTypeFilter(Joke.class,
//						WorkingMemoryOperation.ADD),
//				new WorkingMemoryChangeReceiver() {
//					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//						try {
//							Joke jk = getMemoryEntry(_wmc.address.id, Joke.class);
////							println("found a joke: " + CASTUtils.typeName(jk));
////							for (String type : jk.ice_ids()) {
////								println(type);
////							}
//							
//						} catch (DoesNotExistOnWMException e) {
//							e.printStackTrace();
//						}
//					}
//				});

		
		
//		addChangeFilter(
//				// listen for joke types that are
//				// added that are local to this
//				// subarchitecture
//				ChangeFilterFactory.createLocalTypeFilter(OneLiner.class,
//						WorkingMemoryOperation.ADD),
//				new WorkingMemoryChangeReceiver() {
//					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//						try {
//							OneLiner jk = getMemoryEntry(_wmc.address.id, OneLiner.class);
//							println("found a oneliner");
//						} catch (DoesNotExistOnWMException e) {
//							e.printStackTrace();
//						}
//					}
//				});
		
	}

	private String generatePunchline(String _setup) {
		return "A stick!";
	}

	/**
	 * @param _data
	 */
	private void quip(CASTData<TwoLiner> _data) {

		TwoLiner jk = (TwoLiner) _data.getData();
		// work out a smarty-pants punchline
		String punchline = generatePunchline(jk.setup);
		// time it right
		log("*cough*");
		jk.punchline = punchline;
		// now write this back into working memory
		try {
			overwriteWorkingMemory(_data.getID(), jk);
		} catch (WMException e) {
			e.printStackTrace();
		}

	}

	/**
	 * @param _wmc
	 * @param i
	 */
	private void newTwoLinerAdded(WorkingMemoryChange _wmc) {

		// get the data from working memory and store it
		// with its id
		try {

			// System.out.println("FunnyMan.newTwoLinerAdded()");

			CASTData<TwoLiner> jokeData = getMemoryEntryWithData(_wmc.address,
					TwoLiner.class);
			TwoLiner jk = jokeData.getData();

			// now see if we can provide a punchline
			if (jk.punchline.equals("")) {
				// in this case we need to propose a task to do
				// some processsing
				quip(jokeData);
			}
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}

	}

}
