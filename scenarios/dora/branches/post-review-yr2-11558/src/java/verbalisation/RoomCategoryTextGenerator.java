package verbalisation;

import java.util.ArrayList;
import java.util.HashMap;

import cast.core.CASTUtils;

import comadata.ComaRoom;

public class RoomCategoryTextGenerator implements TextGenerator<ComaRoom> {

	private HashMap<Integer, String> m_lastVerbalisedCategory;

	@Override
	public String toText(ComaRoom _i) {

		// commented out for BHAM Yr2 code camp
		/*
		
		// lazy lazy
		if (m_lastVerbalisedCategory == null) {
			m_lastVerbalisedCategory = new HashMap<Integer, String>();
		}

		// if 1 more than defaults then we have a definite
		if (_i.concepts.length == 3) {
			return textForSingleConcept(_i);
		}
		// we have ambiguity
		else if (_i.concepts.length > 3) {
			return textForMultipleConcepts(_i);
		} else {
			return "";
		} */ return "";
	}

	/**
	 * @param _i
	 * @return
	 */
	private String textForMultipleConcepts(ComaRoom _i) {
		// commented out for BHAM Yr2 code camp
		/*
		StringBuilder sb = new StringBuilder("I infer that room ");
		sb.append(_i.roomId);
		sb.append(" is either a ");

		ArrayList<String> concepts = new ArrayList<String>(_i.concepts.length);
		for (String type : _i.concepts) {
			if (!(type.equals(":PhysicalRoom") || type
					.equals(":Portion_of_space"))) {
				concepts.add(type.substring(1));
			}
		}

		StringBuilder content = new StringBuilder();
		for (int i = 0; i < concepts.size(); i++) {
			content.append(concepts.get(i));
			if (i != concepts.size() - 1) {
				content.append(" or a ");
			}
		}

		String verbContent = content.toString();
		m_lastVerbalisedCategory.put(_i.roomId, verbContent);
		sb.append(verbContent);
		return sb.toString();
		*/ return "";
	}

	/**
	 * @param _i
	 */
	private String textForSingleConcept(ComaRoom _i) {
		// commented out for BHAM Yr2 code camp
		/*
		for (String type : _i.concepts) {
			if (!(type.equals(":PhysicalRoom") || type
					.equals(":Portion_of_space"))) {
				String toVerb = type.substring(1);

				// what did we say last time?
				String lastVerb = m_lastVerbalisedCategory.get(_i.roomId);

				// first shout for this room
				if (lastVerb == null) {
					m_lastVerbalisedCategory.put(_i.roomId, toVerb);
					return CASTUtils.concatenate("I infer that room ",
							_i.roomId, " is a ", toVerb);
				}
				// if it's not the same as last time
				else if (!lastVerb.equals(toVerb)) {
					m_lastVerbalisedCategory.put(_i.roomId, toVerb);
					return CASTUtils.concatenate("I now infer that room ",
							_i.roomId, " is a ", toVerb, " rather than a ",
							lastVerb);
				}
				// else say nothing as it's already been said
			}
		} */
		return "";
	}
}
