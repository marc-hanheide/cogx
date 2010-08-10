package dora;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * A class for providing a random string from a selected set. All strings will
 * be used once before the list is shuffled and they are all providing again in
 * a random order.
 * 
 * @author nah
 * 
 */
public class RandomStringBank {

	private final List<String> m_originalList;

	private final List<String> m_activeList;

	/**
	 * Create a new string bank holding the provided strings.
	 * 
	 * @param _originalList
	 */
	public RandomStringBank(String[] _originalList) {
		m_originalList = Arrays.asList(_originalList);
		m_activeList = new ArrayList<String>(m_originalList.size());
		populateActiveList();
	}

	/**
	 * Add all original strings back into empty active list and shuffle.
	 */
	private void populateActiveList() {
		m_activeList.clear();
		m_activeList.addAll(m_originalList);
		Collections.shuffle(m_activeList);
	}

	/**
	 * Get's the next string at random from the bank.
	 * 
	 * @return
	 */
	public String next() {
		if (m_activeList.isEmpty()) {
			populateActiveList();
		}
		return m_activeList.remove(0);
	}

}
