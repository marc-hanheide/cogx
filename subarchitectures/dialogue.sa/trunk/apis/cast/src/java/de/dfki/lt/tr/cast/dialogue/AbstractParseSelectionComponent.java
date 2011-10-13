// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
// Miroslav Janicek (miroslav.janicek@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

package de.dfki.lt.tr.cast.dialogue;

import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.parseselection.ParseSelector;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.dialogue.util.LFUtils;

public abstract class AbstractParseSelectionComponent<T extends ParseSelector>
extends AbstractDialogueComponent {

	private final T selector;

	public AbstractParseSelectionComponent(T selector) {
		if (selector == null) {
			throw new NullPointerException("selector null");
		}
		this.selector = selector;
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePackedLFs(_wmc);
					}
		});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePackedLFs(_wmc);
					}
		});

	}

	public final T getSelector() {
		return selector;
	}

	private void handlePackedLFs(WorkingMemoryChange _wmc) {
		try {
			PackedLFs plf = getMemoryEntry(_wmc.address, PackedLFs.class);
			if (isFinalized(plf)) {
				addTask(new ProcessingTaskWithData<PackedLFs>(plf) {

					@Override
					public void execute(PackedLFs plf) {
						LogicalForm lf = selector.selectParse(plf);
						if (lf != null) {
							getLogger().info("selected the following LF: [" + LFUtils.lfToString(lf) + "]");
							SelectedLogicalForm slf = new SelectedLogicalForm(lf, plf.phonStringIval, plf.phonStringWordList);
							try {
								addToWorkingMemory(newDataID(), slf);
							}
							catch (AlreadyExistsOnWMException ex) {
								getLogger().error("already exists on WM exception", ex);
							}
						}
						else {
							getLogger().warn("no parse found");
						}
					}

				});
			}
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("subarchitecture component exception", ex);
		}
	}

	private static boolean isFinalized(PackedLFs arg) {
		return arg.finalized == 1;
	}

/*
	public Event createParseErrorEvent() {
		Event e = new Event();
		PrivateEpistemicStatus epst = new PrivateEpistemicStatus();
		epst.agent = "robot";
		e.estatus = epst;
		e.frame = new AbstractFrame();
		CondIndependentDistribs cids = new CondIndependentDistribs();
		cids.distribs = new HashMap<String, ProbDistribution>();
		cids.distribs.put
		return e;
	}
 */

}
