// =================================================================
// Copyright (C) 2011 DFKI GmbH
//
// This library is free software; you can redistribute it and/or 
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

package de.dfki.lt.tr.dialmanagement.data.actions;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;


/**
 * Policy action defined as an event
 * 
 * @author Miroslav Janicek (miroslav.janicek@dfki.de)
 * @version 04/01/2011
 */
public class EventAction extends AbstractAction {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	protected dFormula content;

	/**
	 * Creates an event action, with a unique identifier and
	 * a logical formula
	 * 
	 * @param id the identifier
	 * @param formula the formula
	 */
	public EventAction(String id, dFormula formula) {
		super(id);
		content = formula;
	}


	/**
	 * Do nothing.
	 */
	@Override
	public void fillArguments (HashMap<String,dFormula> arguments) throws DialogueException {
	}


	/**
	 * Returns a logical representation of the event
	 */
	@Override
	public dFormula asFormula() {
		return content;
	}


	/**
	 * Return false.
	 */
	@Override
	public boolean isUnderspecified () {
		return false;
	}


	/**
	 * Return an empty list.
	 */
	@Override
	public Collection<String> getUnderspecifiedArguments () {
		return new LinkedList<String>();
	}


	/**
	 * Returns a string representation of the action
	 */
	@Override
	public String toString() {
		debug("Event ID: " + id);

		String str = "E[" + FormulaUtils.getString(content) + "]";
		return str;
	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[eventaction] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[eventaction] " + s);
		}
	}

}
