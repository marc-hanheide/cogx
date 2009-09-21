package comsys.datastructs.testData;

import java.util.Vector;
import comsys.utils.DialogueMoveUtils;
import comsys.datastructs.comsysEssentials.MoveType;


public class DMTestData extends TestData {

	boolean logging = false;
	
	MoveType moveType ;
	
	public DMTestData() {
		this.string = "";
	}
	
	public DMTestData(String string, String moveType) {
		this.string = string;
		this.moveType = DialogueMoveUtils.convertMoveTypeFromString(moveType);
	}
	
	public void setMoveType(String moveType) {
		log("moveType to insert: " + moveType);
		this.moveType = DialogueMoveUtils.convertMoveTypeFromString(moveType);
		log("successful? " + (this.moveType!=null));
	}
	
	public MoveType getMoveType() {
		return moveType;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[DMTESTDATA] " +  str);
	}
}
