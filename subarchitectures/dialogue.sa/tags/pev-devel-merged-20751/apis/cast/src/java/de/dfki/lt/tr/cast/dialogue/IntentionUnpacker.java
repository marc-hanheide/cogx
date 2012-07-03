package de.dfki.lt.tr.cast.dialogue;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;

public class IntentionUnpacker extends AbstractDialogueComponent {

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(
						PossibleInterpretedIntentions.class,
						WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePossibleInterpretedIntentions(_wmc);
					}
				});
	}

	private void handlePossibleInterpretedIntentions(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithDataAndComponent<WorkingMemoryAddress, IntentionUnpacker>(
				_wmc.address, this) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					unpackMostConfidentIntention(getComponent(), addr);
				} catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}

		});
	}

	/**
	 * /** Unpack the most confident intention from the input address to working
	 * memory.
	 * 
	 * @param _component
	 * @param addr
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 * @throws AlreadyExistsOnWMException
	 */
	public static void unpackMostConfidentIntention(
			ManagedComponent _component, WorkingMemoryAddress addr)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			AlreadyExistsOnWMException {

		PossibleInterpretedIntentions pii = _component.getMemoryEntry(addr,
				PossibleInterpretedIntentions.class);

		unpackMostConfidentIntention(_component, pii);
	}

	/**
	 * Unpack the most confident intention to working memory.
	 * 
	 * 
	 * @param _component
	 * @param pii
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	public static void unpackMostConfidentIntention(
			ManagedComponent _component, PossibleInterpretedIntentions pii)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		WorkingMemoryAddress iintAddr = mostConfidentIntentionAddress(pii);
		unpackIntention(_component, iintAddr, pii);
	}

	/**
	 * Unpack the intention with the given address to working memory.
	 * 
	 * @param _component
	 * @param _intentionAddr
	 * @param _interpretations
	 * @throws AlreadyExistsOnWMException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	public static void unpackIntention(ManagedComponent _component,
			WorkingMemoryAddress _intentionAddr,
			PossibleInterpretedIntentions _interpretations)
			throws AlreadyExistsOnWMException, DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		InterpretedIntention iint = _interpretations.intentions
				.get(_intentionAddr);

		for (WorkingMemoryAddress belAddr : iint.addressContent.values()) {
			if (_interpretations.beliefs.containsKey(belAddr)) {
				dBelief bel = _interpretations.beliefs.get(belAddr);
				_component.getLogger().info(
						"adding a dBelief to " + wmaToString(belAddr) + ": "
								+ BeliefIntentionUtils.beliefToString(bel));
				_component.addToWorkingMemory(belAddr, bel);
			}
		}
		_component.getLogger().info(
				"adding an InterpretedIntention to "
						+ wmaToString(_intentionAddr)
						+ ":\n"
						+ InterpretedUserIntention
								.interpretedIntentionToString(iint));
		_component.addToWorkingMemory(_intentionAddr, iint);
	}

	/**
	 * @param _pii
	 * @return
	 */
	public static InterpretedIntention getMostConfidentIntention(
			PossibleInterpretedIntentions _pii) {
		WorkingMemoryAddress intentionAddress = IntentionUnpacker
				.mostConfidentIntentionAddress(_pii);
		InterpretedIntention mostConfidentIntention = _pii.intentions
				.get(intentionAddress);
		return mostConfidentIntention;
	}

	public static WorkingMemoryAddress mostConfidentIntentionAddress(
			PossibleInterpretedIntentions pii) {
		WorkingMemoryAddress bestAddr = null;
		double best = -1.0;

		for (WorkingMemoryAddress addr : pii.intentions.keySet()) {
			InterpretedIntention iint = pii.intentions.get(addr);

			// System.out.println("most confident intention: "
			// + InterpretedUserIntention
			// .interpretedIntentionToString(iint));

			if (iint.confidence > best) {
				bestAddr = addr;
				best = iint.confidence;
			}
		}

		return bestAddr;
	}

}
