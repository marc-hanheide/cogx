/**
 * @author Team RED
 */

package css11.red.planning;

import cast.architecture.ManagedComponent;
import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTData;
import org.apache.log4j.Logger;
import java.util.Map;
import java.util.HashMap;

import VisionData.SSVisionCommand;
import VisionData.SSVisionCommandType;

import css11.red.planning.slice.Completion;
import css11.red.planning.slice.ManipulationAction;
import css11.red.planning.slice.VisionAction;
import css11.red.planning.slice.VisionRecognizeAction;

public class ActionDispatcher extends ManagedComponent {

	private final static String vision_sa = "vision";

	private Map<WorkingMemoryAddress, WorkingMemoryAddress> vc_to_va
			= new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();

	@Override
	protected void configure(Map<String, String> config) {
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				VisionAction.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleVisionActionAdd(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				ManipulationAction.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleManipulationActionAdd(_wmc);
					}
				});
	}

	// -- VISION STUFF --------------------------------------------------

	protected void handleVisionActionAdd(WorkingMemoryChange wmc) {
		try {
			VisionAction va = getMemoryEntry(wmc.address, VisionAction.class);
			assert (va != null);

			SSVisionCommand vc = visionActionToVisionCommand(va);
			if (vc != null) {

				WorkingMemoryAddress vc_wma = new WorkingMemoryAddress(newDataID(), vision_sa);
				vc_to_va.put(vc_wma, wmc.address);

				// register a change receiver for the object
				addChangeFilter(ChangeFilterFactory.createAddressFilter(
						vc_wma, WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
								public void workingMemoryChanged(WorkingMemoryChange _wmc) {
									handleVisionCommandOverwrite(_wmc);
								}
						});

				// write vc to the vision WM
				addToWorkingMemory(vc_wma, vc);
			}
			else {
				log("failed to convert VisionAction to VisionCommand");
			}
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
		}
	}

	public void handleVisionCommandOverwrite(WorkingMemoryChange wmc) {
		log("woohoo, detected a change on address [" + wmc.address.id + "]");
		try {
			SSVisionCommand vc = getMemoryEntry(wmc.address, SSVisionCommand.class);
			VisionAction va = visionCommandToVisionAction(vc);
			if (va != null) {
				overwriteWorkingMemory(vc_to_va.get(wmc.address), va);
			}
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
		}
	}

	private SSVisionCommand visionActionToVisionCommand(VisionAction va) {
		SSVisionCommand vc = new SSVisionCommand();
		vc.succeed = false;
		vc.objID = "";

		if (va instanceof VisionRecognizeAction) {
			vc.cmd = SSVisionCommandType.SSVRECOGNIZE;
		}
		else {
			vc = null;
		}

		return vc;
	}

	private VisionAction visionCommandToVisionAction(SSVisionCommand vc) {
		switch (vc.cmd) {
			case SSVRECOGNIZESTOP: {
					VisionRecognizeAction rva = new VisionRecognizeAction();
					rva.comp = (vc.succeed ? Completion.SUCCESS : Completion.FAILURE);
					rva.objID = vc.objID;
					return rva;
				}
				
			default:
				return null;
		}
	}

	// -- MANIPULATION STUFF --------------------------------------------


	protected void handleManipulationActionAdd(WorkingMemoryChange wmc) {
		log("UNIMPLEMENTED: handleManipulationActionAdd");
	}
}
