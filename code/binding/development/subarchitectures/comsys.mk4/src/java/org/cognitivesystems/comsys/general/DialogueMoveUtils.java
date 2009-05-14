package org.cognitivesystems.comsys.general;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;

public class DialogueMoveUtils {
 
	public static String convertMoveTypeToString(MoveType mt) {
		if (mt.equals(MoveType.ACCEPT)) 
			return "ACCEPT";
		else if (mt.equals(MoveType.ACTION_DIRECTIVE)) 
			return "ACTION_DIRECTIVE";
		else if (mt.equals(MoveType.CLOSING)) 
			return "CLOSING";
		else if (mt.equals(MoveType.OPENING)) 
			return "OPENING";
		else if (mt.equals(MoveType.QUESTION_W)) 
			return "QUESTION_W";
		else if (mt.equals(MoveType.QUESTION_YN)) 
			return "QUESTION_YN";
		else if (mt.equals(MoveType.REJECT)) 
			return "REJECT";
		else if (mt.equals(MoveType.ASSERT)) 
			return "ASSERT";
		else
			return "none";
	}
	
	public static MoveType convertMoveTypeFromString(String mt) {
		if (mt.equals("ACCEPT")) 
			return MoveType.ACTION_DIRECTIVE;
		else if (mt.equals("ACTION_DIRECTIVE")) 
			return MoveType.ACTION_DIRECTIVE;
		else if (mt.equals("CLOSING")) 
			return MoveType.CLOSING;
		else if (mt.equals("OPENING")) 
			return MoveType.OPENING;
		else if (mt.equals("QUESTION_W")) 
			return MoveType.QUESTION_W;
		else if (mt.equals("QUESTION_YN")) 
			return MoveType.QUESTION_YN;
		else if (mt.equals("REJECT")) 
			return MoveType.REJECT;
		else if (mt.equals("ASSERT")) 
			return MoveType.ASSERT;
		else
			return null;
	}
}
