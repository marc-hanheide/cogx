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
package comedyarch;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

import comedyarch.autogen.DirectorAction;
import comedyarch.autogen.DirectorActionType;
import comedyarch.autogen.Joke;
import comedyarch.autogen.Reaction;

/**
 * A component that watches the world and thinks of things for the director to
 * do.
 * 
 * @author nah
 */
public class AssistantDirector extends ManagedComponent {

	private void handleJoke(WorkingMemoryChange _wmc) {
		try {
			// get the joke from working memory to see what
			// we can do

			Joke joke = getMemoryEntry(_wmc.address, Joke.class);

			// now see if the joke is complete!
			if ((!joke.setup.equals("")) && (!joke.punchline.equals(""))) {

				println("is that a joke I hear?!");

				// if it's tell the director to check it out
				DirectorAction action = new DirectorAction(
						DirectorActionType.AskTheAudience, _wmc.address);

				// by writing the action to working memory
				addToWorkingMemory(newDataID(), action);
			}
			
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged
	 * (cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
	 */
	@Override
	public void start() {
		WorkingMemoryChangeReceiver jokeReceiver = new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				log(CASTUtils.toString(_wmc));
				handleJoke(_wmc);
			}
		};

		// look for jokes in the stage subarchiture... oops ugly hard coded
		// string

		addChangeFilter(ChangeFilterFactory.createChangeFilter(Joke.class,
				WorkingMemoryOperation.OVERWRITE, "", "", "stage.subarch",
				FilterRestriction.ALLSA), jokeReceiver);

		WorkingMemoryChangeReceiver reactionReceiver = new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				handleReaction(_wmc);
			}
		};

		// and look for reactions when they're added
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				Reaction.class, WorkingMemoryOperation.ADD), reactionReceiver);

	}

	/**
	 * @param _wmc
	 * @param i
	 * @throws CASTOntologyException
	 */
	private void handleReaction(WorkingMemoryChange _wmc) {
		println("is that a reaction I hear?!");

		// let's propose a task goal to see what the
		// audience thinks of that shall we!
		// if it's tell the director to check it out

		DirectorAction action = new DirectorAction(
				DirectorActionType.CheckTheReaction, _wmc.address);

		// by writing the action to working memory
		try {
			addToWorkingMemory(newDataID(), action);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}

	}

}
