package coma.components;

import java.util.Map;

import comadata.ComaRoom;

import comsys.datastructs.comsysEssentials.ContentPlanningGoal;
import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class ComaVerbTester extends ManagedComponent {

	private String m_comsys_SA;
	
	public void configure(Map<String, String> args) {
		log("entered configure()");
		m_comsys_SA = "";
		if (args.containsKey("--comsys")) {
			log("args comntains key --comsys");
			m_comsys_SA=args.get("--comsys");
		}
		log("comsys SA = " + m_comsys_SA);
	}

	
	public void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ComaRoom.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				processAddedRoom(_wmc);
			};
		});
	}
	
	public void runComponent() {
		LogicalForm _greetingLF = LFUtils.convertFromString("@d1:dvp(c-goal ^ <SpeechAct>greeting)");
		
		log("created LF: "+ _greetingLF);
		
		ContentPlanningGoal _cpGoal = new ContentPlanningGoal(newDataID(), _greetingLF);
		
		try {
			addToWorkingMemory(new WorkingMemoryAddress(newDataID(), m_comsys_SA), _cpGoal);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		log("wrote content planning goal to WM: " + _cpGoal);
	}
	
	
	private void processAddedRoom(WorkingMemoryChange _wmc) {
		LogicalForm _whatLF = LFUtils.convertFromString("@d1:dvp(c-goal  ^ <SpeechAct>question ^  <Content>(e1:ascription ^ <Target>(b2:entity ^ <Salient>true ^ room)  ^ <Type>(b3:ont-entity ^ room ^ <Questioned>true)))");
		ContentPlanningGoal _cpg = new ContentPlanningGoal(newDataID(), _whatLF);
		try {
			addToWorkingMemory(new WorkingMemoryAddress(newDataID(), m_comsys_SA), _cpg);
			log("wrote content planning goal to WM " + _cpg + " with LF " + _whatLF);
		} catch (AlreadyExistsOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	
	
}
