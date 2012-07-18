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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * An action that removes the variable in the shared dialogue state
 * 
 * @author Miroslav Janicek (miroslav.janicek@dfki.de)
 * @version 04/01/2011
 *
 */
public class RemoveVariableAction extends AbstractAction {

	protected String label;
	
	/**
	 * Construct a new ClearVariableAction.
	 * 
	 * @param id the action identifier
	 * @param label the label
	 */
	public RemoveVariableAction(String id, String label) {
		super(id);
		this.label = label;
	}

	/**
	 * Return the label of the variable to be removed.
	 */
	public String getLabel() {
		return label;
	}


	/**
	 * Return null.
	 */
	@Override
	public dFormula asFormula() {
		return null;
	}


	/**
	 * Do nothing.
	 */
	@Override
	public void fillArguments(HashMap<String, dFormula> arguments)
			throws DialogueException {
	}


	/**
	 * Return an empty list.
	 */
	@Override
	public Collection<String> getUnderspecifiedArguments() {
		return new LinkedList<String>();
	}


	/**
	 * Return false.
	 */
	@Override
	public boolean isUnderspecified() {
		return false;
	}

	
	/**
	 * Returns a string representation of the action
	 */
	@Override
	public String toString() {
		return "rmvar(" + label + ")";
	}

}
