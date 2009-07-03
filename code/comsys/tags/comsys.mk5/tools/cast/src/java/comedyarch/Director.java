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

import java.util.Map;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

import comedyarch.autogen.DirectorAction;
import comedyarch.autogen.DirectorActionType;
import comedyarch.autogen.TwoLiner;
import comedyarch.autogen.Reaction;

/**
 * @author nah
 */
public class Director extends ManagedComponent {

//	private boolean m_testing;
	private String m_audienceSA;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				DirectorAction.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						// log(CASTUtils.toString(_wmc));
						takeAction(_wmc);
					}
				});
	}

	/**
	 * @param _address
	 */
	private void takeAction(WorkingMemoryChange _actionChange) {
		// get the action from wm
		try {
			DirectorAction action = getMemoryEntry(_actionChange.address,
					DirectorAction.class);
			switch (action.action.value()) {
			case DirectorActionType._AskTheAudience:
				askTheAudience(action.address);
				break;
			case DirectorActionType._CheckTheReaction:
				checkTheReaction(action.address);
				break;
			default:
				log("Unknown action type");
				break;
			}

			// // now delete the action itself from our sa wm
			// debug("deleting action at: " + CASTUtils.toString(_wma));
			deleteFromWorkingMemory(_actionChange.address);
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param _reactionAddress
	 * @throws PermissionException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException 
	 * @throws SubarchitectureProcessException
	 */
	private void checkTheReaction(WorkingMemoryAddress _reactionAddress)
			throws DoesNotExistOnWMException, PermissionException, UnknownSubarchitectureException {

		// get the finished masterpiece
		String reaction = getMemoryEntry(_reactionAddress, Reaction.class).react;

		println("and the audience says...");
		println(reaction);

		// and erase it from the memory of the poor jokers
		deleteFromWorkingMemory(_reactionAddress);

		// if(m_testing) {
		// if(reaction.equals("YAY!")) {
		// System.exit(CAST_TEST_PASS.value);
		// }
		// else {
		// System.exit(CAST_TEST_FAIL.value);
		// }
		// }
	}

	/**
	 * @param _jokeAddress
	 * @throws PermissionException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException 
	 * @throws SubarchitectureProcessException
	 */
	private void askTheAudience(WorkingMemoryAddress _jokeAddress)
			throws DoesNotExistOnWMException, PermissionException,
			AlreadyExistsOnWMException, UnknownSubarchitectureException {

		// assert(existsOnWorkingMemory(_jokeAddress));

		println("someone told a joke! ");

		// get the finished masterpiece
		TwoLiner joke = getMemoryEntry(_jokeAddress, TwoLiner.class);

		println("let's see what the audience thinks of...");
		println("Q: " + joke.setup);
		println("A: " + joke.punchline);

		// write it to the audience subarchitecture
		addToWorkingMemory(newDataID(), m_audienceSA, joke);

		// and erase it from the memory of the poor jokers
		deleteFromWorkingMemory(_jokeAddress);
	}

	@Override
	public void configure(Map<String, String> _config) {
		String asa = _config.get("--audience");
		if (asa != null) {
			m_audienceSA = asa;
		} else {
			throw new RuntimeException(
					"Missing --audience parameter for audience subarch address");
		}
	}

}
