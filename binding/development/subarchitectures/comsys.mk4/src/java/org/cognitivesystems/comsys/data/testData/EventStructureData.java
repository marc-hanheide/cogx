package org.cognitivesystems.comsys.data.testData;

import java.util.Vector;
import org.cognitivesystems.comsys.general.DialogueMoveUtils;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;


public class EventStructureData extends TestData {

	boolean logging = false;
	
	String eventtype ="";
	String statetype = "";
	
	public EventStructureData() {
		this.string = "";
	}
	
	public EventStructureData(String string, String eventtype, String statetype) {
		this.string = string;
		this.eventtype = eventtype;
		this.statetype = statetype;
	}
	
	public void setEventType(String eventtype) {
		this.eventtype = eventtype;
	}
	
	public void setStateType(String statetype) {
		this.statetype = statetype;
	}
	
	public String getEventType() {
		return eventtype;
	}
	
	public String getStateType() {
		return statetype;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[EVENT STRUCTURE DATA] " +  str);
	}
}
