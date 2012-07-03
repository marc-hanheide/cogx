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

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.IntentionRealization;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;

public class CommunicativeIntentionRealizer
extends AbstractDialogueComponent {

	public final static int DEFAULT_TIMEOUT = 250;
	private int timeout = DEFAULT_TIMEOUT;

	private IntentionRealization ireal;
	private String rulesetFile = "/dev/null";
	private String dumpFile = "/tmp/belief-model.abd";
	private List<String> files = new LinkedList<String>();
	private HashMap<String, EpistemicObject> epObjs = new HashMap<String, EpistemicObject>();

	private String abd_serverName = "AbducerServer";
	private int abd_port = 9100;
	private String abd_endpoints = "default -p " + abd_port;

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);
		if (args.containsKey("--ruleset")) {
			rulesetFile = args.get("--ruleset");
		}
		if (args.containsKey("--timeout")) {
			String timeoutStr = args.get("--timeout");
			timeout = Integer.parseInt(timeoutStr);
		}
		if (args.containsKey("--dumpfile")) {
			dumpFile = args.get("--dumpfile");
		}
		
		String abducerHost = args.get("--abd-host");
		if (abducerHost != null) {
			abd_endpoints = "default -h " + abducerHost + " -p " + abd_port;
		}
		
		if (rulesetFile != null) {
			try {
				BufferedReader f = new BufferedReader(new FileReader(rulesetFile));
				String parentAbsPath = (new File((new File(rulesetFile)).getParent()).getCanonicalPath());
				if (parentAbsPath == null) {
					parentAbsPath = "";  // rulefile is in `/'
				}
				log("will be looking for abducer rulefiles in `" + parentAbsPath + "'");
				String file = null;
				while ((file = f.readLine()) != null) {
					file = parentAbsPath + File.separator + file;
					files.add(file);
				}
				f.close();
			}
			catch (FileNotFoundException ex) {
				getLogger().error("ruleset file not found", ex);
			}
			catch (IOException ex) {
				getLogger().error("I/O exception while reading files from list", ex);
			}
		}
	}

	@Override
	public void onStart() {
		super.onStart();

		ireal = new IntentionRealization(abd_serverName, abd_endpoints, new IdentifierGenerator() {
			@Override
			public String newIdentifier() {
				return newDataID();
			}
		}, timeout);

		initialiseContext();
		files.add(dumpFile);

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleCommunicativeIntention(_wmc);
					}
		});
	}

	private void handleCommunicativeIntention(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					CommunicativeIntention cit = getMemoryEntry(addr, CommunicativeIntention.class);
					if (cit.intent.content.size() > 1) {
						getLogger().warn("don't know how to handle a comm. intention with " + cit.intent.content.size() + " alternative contents => ignoring it");
					}
					else {
						IntentionalContent itnc = cit.intent.content.get(0);
						if (itnc.agents.size() == 1 && itnc.agents.get(0).equals(IntentionManagementConstants.thisAgent)) {
							getLogger().info("got a private comm. intention, will try to realise it");

							getLogger().debug("processing a communicative intention " + wmaToString(addr) + ", id=" + cit.intent.id + ": "
									+ BeliefIntentionUtils.intentionToString(cit.intent));
							initialiseContext();

							ContentPlanningGoal protoLF = ireal.epistemicObjectsToProtoLF(addr, cit.intent, new LinkedList<dBelief>());
							if (protoLF != null) {
								getLogger().info("adding proto-LF to working memory: " + LFUtils.lfToString(protoLF.lform));
								addToWorkingMemory(newDataID(), protoLF);
							}
							else {
								getLogger().warn("no proto-LF generated for the comm. intention");
							}

						}
						else {
							log("ignoring a comm. intention that is not the robot's");
						}
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarch component exception", ex);
				}
			}
			
		});
	}

	private void initialiseContext() {
		getLogger().debug("initialising context");
		ireal.clearContext();
		for (String f : files) {
			getLogger().debug("reading file " + f);
			ireal.loadFile(f);
		}
	}

}
