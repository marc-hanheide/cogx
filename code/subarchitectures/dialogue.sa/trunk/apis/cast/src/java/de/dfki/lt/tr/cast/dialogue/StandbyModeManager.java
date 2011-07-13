package de.dfki.lt.tr.cast.dialogue;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
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
extends ManagedComponent
{
	private WorkingMemoryAddress current = null;
	private String unlockPhrase = "resume listening";
	private double unlockThreshold = 0.7;

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(StandbyMode.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleStandby(_wmc);
					}
				});
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(StandbyMode.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleStandby(_wmc);
					}
				});
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(InitialPhonString.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleInitialPhonString(_wmc);
					}
				});
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(InitialNoise.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleInitialNoise(_wmc);
					}
				});
	}

	private void handleStandby(WorkingMemoryChange _wmc) {
		if (_wmc.operation == WorkingMemoryOperation.ADD) {
			log("StandbyMode ADD -> turning on local standby mode");
			current = _wmc.address;
		}
		else if (_wmc.operation == WorkingMemoryOperation.DELETE) {
			log("StandbyMode DELETE -> turning off local standby mode");
			current = null;
		}
		else {
			log("ignoring " + _wmc.operation.toString() + " event for StandbyMode");
		}
	}

	private void handleInitialPhonString(WorkingMemoryChange _wmc) {
		try {
			InitialPhonString ips = getMemoryEntry(_wmc.address, InitialPhonString.class);

			if (onStandby()) {
				if (isUnlockPhonString(ips.ps)) {
					log("yippieh! no more silence!");
					deleteFromWorkingMemory(current);
					CommunicativeIntention cit = generateResumedIntention();
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
		catch (AlreadyExistsOnWMException ex) {
			ex.printStackTrace();
		}
		catch (DoesNotExistOnWMException ex) {
			ex.printStackTrace();
		}
		catch (UnknownSubarchitectureException ex) {
			ex.printStackTrace();
		}
		catch (PermissionException ex) {
			ex.printStackTrace();
		}
	}

	private void handleInitialNoise(WorkingMemoryChange _wmc) {
		try {
			InitialNoise in = getMemoryEntry(_wmc.address, InitialNoise.class);
			if (onStandby()) {
				log("ignoring a Noise");
			}
			else {
				log("forwarding a Noise");
				addToWorkingMemory(newDataID(), in.n);
			}
		}
		catch (AlreadyExistsOnWMException ex) {
			ex.printStackTrace();
		}
		catch (DoesNotExistOnWMException ex) {
			ex.printStackTrace();
		}
		catch (UnknownSubarchitectureException ex) {
			ex.printStackTrace();
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


	private CommunicativeIntention generateResumedIntention() {
		Intention it = new Intention();
		it.id = newDataID();
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

	@Override
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		if (_config.containsKey("--unlock-phrase")) {
			unlockPhrase = _config.get("--unlock-phrase");
		}
		if (_config.containsKey("--unlock-threshold")) {
			try {
				unlockThreshold = Float.parseFloat(_config.get("--unlock-threshold"));
			}
			catch (NumberFormatException e) {
				log("wrong format for threshold");
			}
		}
	}
}
