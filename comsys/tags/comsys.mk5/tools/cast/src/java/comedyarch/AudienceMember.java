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

package comedyarch;

import java.util.Map;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

import comedyarch.autogen.TwoLiner;
import comedyarch.autogen.Reaction;

/**
 * @author nah
 */
public class AudienceMember extends ManagedComponent {

	private String m_defaultReaction;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				TwoLiner.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						newTwoLinerAdded(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				TwoLiner.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						newTwoLinerAdded(_wmc);
					}
				});
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * cast.core.components.CASTProcessingComponent#configure(java.util.Properties
	 * )
	 */
	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--reaction")) {
			m_defaultReaction = _config.get("--reaction");
		} else {
			m_defaultReaction = "BOO!";
		}
	}

	private Reaction generateAudienceReaction(TwoLiner _joke) {
		return new Reaction(m_defaultReaction);
	}

	/**
	 * @param _wmc
	 * @param i
	 */
	private void newTwoLinerAdded(WorkingMemoryChange _wmc) {
		// get the joke from working memory to evaluate
		// get the id of the working memory entry
		String id = _wmc.address.id;

		try {
			if (_wmc.address.subarchitecture.equals(getSubarchitectureID())) {
				// get the data from working memory and store it
				// with its id
				TwoLiner joke = getMemoryEntry(id, TwoLiner.class);
				println("(takes deep breath)");
				Reaction reaction = generateAudienceReaction(joke);
				// let's tell the world!
				addToWorkingMemory(newDataID(), reaction);
			} else {

				log("I'll have a cheeky peek into subarch: "
						+ _wmc.address.subarchitecture);
				TwoLiner joke = getMemoryEntry(_wmc.address, TwoLiner.class);
				log("I heard a joke:");
				log(joke.setup);
				log(joke.punchline);
			}
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}

	}

}
