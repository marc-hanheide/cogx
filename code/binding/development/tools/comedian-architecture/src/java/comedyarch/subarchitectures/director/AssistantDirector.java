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
package comedyarch.subarchitectures.director;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

import comedyarch.autogen.DirectorAction;
import comedyarch.autogen.DirectorActionType;
import comedyarch.autogen.Joke;

/**
 * A component that watches the world and thinks of things for the director to
 * do.
 * 
 * @author nah
 */
public class AssistantDirector extends PrivilegedManagedProcess {

	/**
	 * @param _id
	 */
	public AssistantDirector(String _id) {
		super(_id);
//		setOntology(ComedyOntologyFactory.getOntology());
	}

	private void handleJoke(WorkingMemoryChange _wmc) {
		try {
			// get the joke from working memory to see what
			// we can do

			// get the data from working memory and store it
			// with its id
			CASTData<?> jokeData = getWorkingMemoryEntry(_wmc.m_address);

			// because we've checked the ontological type
			// above, this should be safe
			Joke joke = (Joke) jokeData.getData();

			// now see if the joke is complete!
			if ((!joke.m_setup.equals("")) && (!joke.m_punchline.equals(""))) {

				println("is that a joke I hear?!");

				// if it's tell the director to check it out
				DirectorAction action = new DirectorAction(
						DirectorActionType.AskTheAudience, _wmc.m_address);

				// by writing the action to working memory
				addToWorkingMemory(newDataID(), action);

			}
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
	 */
	@Override
	public void start() {
		super.start();

		try {

			WorkingMemoryChangeReceiver jokeReceiver = new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					log(CASTUtils.toString(_wmc));
					handleJoke(_wmc);
				}
			};

			// look for jokes in the stage subarchiture
			// addChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.ADD, "", "",
			// ComedyClient.STAGE_SUBARCH, FilterRestriction.ALL_SA,
			// jokeReceiver);

			addChangeFilter(ChangeFilterFactory.createChangeFilter(Joke.class,
					WorkingMemoryOperation.OVERWRITE, "", "", "stage.subarch",
					FilterRestriction.ALL_SA), jokeReceiver);

			WorkingMemoryChangeReceiver reactionReceiver = new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handleReaction(_wmc);
				}
			};

			// and look for reactions when they're added
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					String.class, WorkingMemoryOperation.ADD), reactionReceiver);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/**
	 * @param _wmc
	 * @param i
	 * @throws CASTOntologyException
	 */
	private void handleReaction(WorkingMemoryChange _wmc) {
		try {
			println("is that a reaction I hear?!");

			// let's propose a task goal to see what the
			// audience thinks of that shall we!
			// if it's tell the director to check it out

			DirectorAction action = new DirectorAction(
					DirectorActionType.CheckTheReaction, _wmc.m_address);

			// by writing the action to working memory
			addToWorkingMemory(newDataID(), action);

		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	@Override
	protected void taskAdopted(String _taskID) {
	}

	@Override
	protected void taskRejected(String _taskID) {
	}

}
