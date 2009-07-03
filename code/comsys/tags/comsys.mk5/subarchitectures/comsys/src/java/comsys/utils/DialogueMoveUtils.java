package comsys.utils;

import comsys.datastructs.comsysEssentials.*;

public class DialogueMoveUtils {
 
	public static String convertMoveTypeToString(MoveType mt) {
		if (mt.equals(MoveType.ACCEPT)) 
			return "ACCEPT";
		else if (mt.equals(MoveType.ACTIONDIRECTIVE)) 
			return "ACTION_DIRECTIVE";
		else if (mt.equals(MoveType.CLOSING)) 
			return "CLOSING";
		else if (mt.equals(MoveType.OPENING)) 
			return "OPENING";
		else if (mt.equals(MoveType.QUESTIONW)) 
			return "QUESTION_W";
		else if (mt.equals(MoveType.QUESTIONYN)) 
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
			return MoveType.ACTIONDIRECTIVE;
		else if (mt.equals("ACTION_DIRECTIVE")) 
			return MoveType.ACTIONDIRECTIVE;
		else if (mt.equals("CLOSING")) 
			return MoveType.CLOSING;
		else if (mt.equals("OPENING")) 
			return MoveType.OPENING;
		else if (mt.equals("QUESTION_W")) 
			return MoveType.QUESTIONW;
		else if (mt.equals("QUESTION_YN")) 
			return MoveType.QUESTIONYN;
		else if (mt.equals("REJECT")) 
			return MoveType.REJECT;
		else if (mt.equals("ASSERT")) 
			return MoveType.ASSERT;
		else
			return null;
	}
}
