// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.cast.dialogue;

// Java
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Map;

// CAST
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

// Dialogue API CAST

//Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.Noise;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.asr.InitialPhonString;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;

//Dialogue API 
import de.dfki.lt.tr.dialogue.util.DialogueChatWindow;

// META
import de.dfki.lt.tr.meta.TRResultListener;

public class DialogueGUI
extends AbstractDialogueComponent
implements TRResultListener {

	private final DialogueChatWindow gui;

	public DialogueGUI() {
		gui = new DialogueChatWindow();
		gui.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});
		gui.pack();
	}

	@Override
	protected void onConfigure(Map<String, String> args) {
		gui.setVisible(true);
		gui.registerNotification(this);
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(SpokenOutputItem.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleSOIAdd(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(PhonString.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePhonStringAdd(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Noise.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleNoiseAdd(_wmc);
					}
				});
	}

	private void handleSOIAdd(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {
			@Override
			public void execute(WorkingMemoryChange wmc) {
				try {
					gui.publishSOI(getMemoryEntry(wmc.address, SpokenOutputItem.class));
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarchitecture component exception", ex);
				}
			}
		});
	}

	private void handlePhonStringAdd(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {
			@Override
			public void execute(WorkingMemoryChange wmc) {
				try {
					gui.publishPhonString(getMemoryEntry(wmc.address, PhonString.class));
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarchitecture component exception", ex);
				}
			}
		});
	}

	private void handleNoiseAdd(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {
			@Override
			public void execute(WorkingMemoryChange wmc) {
				try {
					gui.publishNoise(getMemoryEntry(wmc.address, Noise.class));
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarchitecture component exception", ex);
				}
			}
		});
	}

	@Override
	public void notify(Object result) {
		InitialPhonString ips = (InitialPhonString) result;
		addTask(new ProcessingTaskWithData<InitialPhonString>(ips) {
			@Override
			public void execute(InitialPhonString ips) {
				try {
					addToWorkingMemory(newDataID(), ips);
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarchitecture component exception", ex);
				}
			}
		});
	}

}
