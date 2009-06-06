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

import comedyarch.autogen.Joke;

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
				newJokeAdded(_wmc);
			}
		};

		// add this receiver to listen for changes
		addChangeFilter(
		// listen for joke types that are
				// added that are local to this
				// subarchitecture
				ChangeFilterFactory.createLocalTypeFilter(Joke.class,
						WorkingMemoryOperation.ADD), receiver);

	}

	
	private String generatePunchline(String _setup) {
		return "A stick!";
	}

	/**
	 * @param _data
	 */
	private void quip(CASTData<Joke> _data) {

		Joke jk = (Joke) _data.getData();
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
	private void newJokeAdded(WorkingMemoryChange _wmc) {

		// get the data from working memory and store it
		// with its id
		try {
			CASTData<Joke> jokeData = getMemoryEntryWithData(_wmc.address, Joke.class);
			Joke jk = jokeData.getData();

//			ArrayList<Joke> entries = new ArrayList<Joke>();
//			getMemoryEntries(Joke.class, entries, 1);
//			if(entries.isEmpty()) {
//				log("no working memory entries");
//				return;
//			}
//			Joke jk = entries.get(0);

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
