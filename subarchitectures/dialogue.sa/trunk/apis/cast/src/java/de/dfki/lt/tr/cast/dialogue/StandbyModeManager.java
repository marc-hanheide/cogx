package de.dfki.lt.tr.cast.dialogue;

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
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.slice.StandbyMode;
import de.dfki.lt.tr.dialogue.slice.asr.InitialNoise;
import de.dfki.lt.tr.dialogue.slice.asr.InitialPhonString;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class StandbyModeManager
extends AbstractDialogueComponent {

	public static final String DEFAULT_UNLOCK_PHRASE = "resume listening";
	public static final double DEFAULT_UNLOCK_THRESHOLD = 0.7;

	private WorkingMemoryAddress current = null;
	private String unlockPhrase = "resume listening";
	private double unlockThreshold = 0.7;

	public StandbyModeManager(String unlockPhrase, double unlockThreshold) {
		this.unlockPhrase = unlockPhrase;
		this.unlockThreshold = unlockThreshold;
		current = null;
	}

	public StandbyModeManager() {
		this(DEFAULT_UNLOCK_PHRASE, DEFAULT_UNLOCK_THRESHOLD);
	}

	@Override
	protected void onConfigure(Map<String, String> _config) {
		super.onConfigure(_config);
		if (_config.containsKey("--unlock-phrase")) {
			unlockPhrase = _config.get("--unlock-phrase");
		}
		if (_config.containsKey("--unlock-threshold")) {
			try {
				unlockThreshold = Float.parseFloat(_config.get("--unlock-threshold"));
			}
			catch (NumberFormatException ex) {
				getLogger().error("wrong format for threshold", ex);
			}
		}
	}

	@Override
	protected void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(StandbyMode.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {
							@Override
							public void execute(WorkingMemoryAddress addr) {
								log("StandbyMode ADD -> turning on local standby mode");
								current = addr;
							}
						});
					}
				});
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(StandbyMode.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithoutData() {
							@Override
							public void execute() {
								log("StandbyMode DELETE -> turning off local standby mode");
								current = null;
							}
						});
					}
				});
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(InitialPhonString.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {
							@Override
							public void execute(WorkingMemoryAddress addr) {
								handleInitialPhonString(addr);
							}
						});
					}
				});
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(InitialNoise.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {
							@Override
							public void execute(WorkingMemoryAddress addr) {
								handleInitialNoise(addr);
							}
						});
					}
				});
	}

	private void handleInitialPhonString(WorkingMemoryAddress addr) {
		try {
			InitialPhonString ips = getMemoryEntry(addr, InitialPhonString.class);

			if (onStandby()) {
				if (isUnlockPhonString(ips.ps)) {
					log("yippieh! no more silence!");
					deleteFromWorkingMemory(current);
					CommunicativeIntention cit = generateResumedIntention(newDataID());
					if (cit != null) {
						addToWorkingMemory(newDataID(), cit);
					}
				}
				else {
					log("ignoring a PhonString (on standby)");
				}
			}
			else {
				log("forwarding a PhonString");
				addToWorkingMemory(ips.ps.id, ips.ps);
			}
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("subarchitecture component exception", ex);
		}
	}

	private void handleInitialNoise(WorkingMemoryAddress addr) {
		try {
			InitialNoise in = getMemoryEntry(addr, InitialNoise.class);
			if (onStandby()) {
				log("ignoring a Noise");
			}
			else {
				log("forwarding a Noise");
				addToWorkingMemory(newDataID(), in.n);
			}
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("subarchitecture component exception", ex);
		}
	}

	private boolean isUnlockPhonString(PhonString ps) {
		if (ps.confidenceValue >= unlockThreshold) {
			if (ps.wordSequence.equals(unlockPhrase)) {
				return true;
			}
		}
		return false;
	}

	private boolean onStandby() {
		return current != null;
	}


	private static CommunicativeIntention generateResumedIntention(String id) {
		Intention it = new Intention();
		it.id = id;
		it.estatus = new PrivateEpistemicStatus("self");
		it.content = new LinkedList<IntentionalContent>();

		// it's the robot's intention
		List<String> ags = new LinkedList<String>();
		ags.add("self");

		// construct the postcondition (the state)
		ComplexFormula inState = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
		inState.forms.add(new ElementaryFormula(0, "dialogue-resumed"));

		ModalFormula state = new ModalFormula(0, "state", inState);

		ComplexFormula post = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
		post.forms.add(state);

		IntentionalContent itc = new IntentionalContent(ags, new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj), post, 1.0f);
		it.content.add(itc);
		it.frame = new AbstractFrame();

		CommunicativeIntention cit = new CommunicativeIntention(it);
		return cit;
	}

}
