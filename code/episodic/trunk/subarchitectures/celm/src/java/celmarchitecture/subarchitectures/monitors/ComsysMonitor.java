package celmarchitecture.subarchitectures.monitors;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;


import java.util.Date;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.BALTTimeConverter;
import celmarchitecture.global.EventTypeNames;
import celmarchitecture.global.GlobalSettings;


/** 
 *  ComsysMonitor is a simple monitor process for changes on Comsys WM 
 *  which might be interesting.
 *  @author Dennis Stachowicz
 */
public class ComsysMonitor extends SimpleAbstractWMMonitor {

 
    public ComsysMonitor(String _id) {
        super(_id);
    }
    
       
    @Override
    public void start() {
        super.start();

        try {
	    WorkingMemoryChangeReceiver wmcrProcessEvent = new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processEvent(_wmc);			
                    }
		};


	    addGlobalAddOverwriteFilter(PhonString.class, wmcrProcessEvent);
	    
	    // temporarily deactivated (possible null pointers in DialogueMoves)
	    // addGlobalAddOverwriteFilter(DialogueMove.class, wmcrProcessEvent);
	    
	    addGlobalAddOverwriteFilter(SpokenOutputItem.class, wmcrProcessEvent);

	    // communicative goals???
	    
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }




    protected void processEvent(WorkingMemoryChange _wmc) {

	
        try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof PhonString) {

		PhonString ps = (PhonString) data;
		EventSpecificFeatures esf = new EventSpecificFeatures(4);
		
		esf.addKeyValuePair("id", "" + ps.id);
		esf.addKeyValuePair("wordSequence", "" + ps.wordSequence);
		esf.addKeyValuePair("length", "" + ps.length);
		esf.addKeyValuePair("confidenceValue", "" + ps.confidenceValue);		
		addPartialEvent(EventTypeNames.phonString, 
				EventSpecificBinaryDataIO.objectToByteArray(ps), 
				null, 
				null, 
				null, 
				esf);
	    }
	    else if (data instanceof DialogueMove) {
		
		DialogueMove dm = (DialogueMove) data;
		
		// default type name
		String eventTypeName = EventTypeNames.dialogueMove;
		
		EventSpecificFeatures esf = new EventSpecificFeatures(4);
		esf.addKeyValuePair("dmId", "" + dm.dmId);
		esf.addKeyValuePair("mType", "" + dm.mType);
		
		switch (dm.mType.value()) {		    
		    
		    // OPENING gets a special treatment here
		    // with dialogueMoveOpening being a subtype of dialogueMove
		    case MoveType._OPENING: 
			eventTypeName = EventTypeNames.dialogueMoveOPENING;
			esf.addKeyValuePair("mTypeAsString", "OPENING");
			break;
						
		   // The does not get the same special treatment yet...
		    case MoveType._CLOSING: 
			esf.addKeyValuePair("mTypeAsString", "CLOSING");
			break;
		    
		    
		    case MoveType._ASSERT: 
			esf.addKeyValuePair("mTypeAsString", "ASSERT");
			break;
		    case MoveType._ACTION_DIRECTIVE: 
			esf.addKeyValuePair("mTypeAsString", "ACTION_DIRECTIVE");
			break;
		    case MoveType._QUESTION_W: 
			esf.addKeyValuePair("mTypeAsString", "QUESTION_W");
			break;
		    case MoveType._QUESTION_YN: 
			esf.addKeyValuePair("mTypeAsString", "QUESTION_YN");
			break;
						

			
		    case MoveType._ACCEPT: 
			esf.addKeyValuePair("mTypeAsString", "ACCEPT");
			break;
		    case MoveType._REJECT: 
			esf.addKeyValuePair("mTypeAsString", "REJECT");
			break;	
		
		    default:
			esf.addKeyValuePair("mTypeAsString", "--unknown--");
			break;
		}
		
		// esf.addKeyValuePair("SDRSFormulaId1", "" + dm.SDRSFormulaId1);
		// esf.addKeyValuePair("SDRSFormulaId2", "" + dm.SDRSFormulaId2);
		
		addPartialEvent(eventTypeName, 
				EventSpecificBinaryDataIO.objectToByteArray(dm), 
				null, 
				null, 
				null, 
				esf);
	    }
	    else if (data instanceof SpokenOutputItem) {
		SpokenOutputItem soi = (SpokenOutputItem) data;
		EventSpecificFeatures esf = new EventSpecificFeatures(2);
		esf.addKeyValuePair("soiid", "" + soi.soiid);
		esf.addKeyValuePair("phonString", "" + soi.phonString);
		addPartialEvent(EventTypeNames.spokenOutputItem, 
				EventSpecificBinaryDataIO.objectToByteArray(soi), 
				null, 
				null, 
				null, 
				esf);	
	    }

	    // -----  communicative goal not found in idl -----
	    
        }
	catch (java.io.IOException e)  {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }
    
}