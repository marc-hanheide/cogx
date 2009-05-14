/*
 * Comedian example code to demonstrate CAST functionality.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package comedyarch.subarchitectures.memory;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTTypedData;

import comedyarch.autogen.JokeBook;

/**
 * @author nah
 */
public class JokeBookReader extends PrivilegedManagedProcess {

	/**
	 * @param _id
	 */
	public JokeBookReader(String _id) {
		super(_id);
//		setOntology(ComedyOntologyFactory.getOntology());
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.CAST.WorkingMemoryChange[])
	 */
	@Override
	public void start() {

		super.start();

		WorkingMemoryChangeReceiver receiver = new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				readJokeBook(_wmc);
			}
		};

		try {
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					JokeBook.class, WorkingMemoryOperation.ADD), receiver);
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					JokeBook.class, WorkingMemoryOperation.OVERWRITE), receiver);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/**
	 * @param _wmc
	 * @param i
	 */
	private void readJokeBook(WorkingMemoryChange _wmc) {
		// get the joke book from working memory to evaluate
		try {
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;

			// get the data from working memory
			CASTTypedData jokeBookData = getWorkingMemoryEntry(id);
			JokeBook book = (JokeBook) jokeBookData.getData();
			println("I've found a joke book called: " + book.m_title);
			for (int j = 0; j < book.m_jokeCount; j++) {
				println("\tQ: " + book.m_jokes[j].m_setup);
				println("\tA: " + book.m_jokes[j].m_punchline);
			}

		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	public void runComponent() {

	}

}
