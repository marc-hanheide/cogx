///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2002-5 Jason Baldridge and University of Edinburgh (Michael White)
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////

package opennlp.ccg.synsem;

import opennlp.ccg.unify.*;
import gnu.trove.*;

/**
 * A variable representing a stack of arguments
 * 
 * @author Jason Baldridge
 * @author Michael White
 * @version $Revision: 1.6 $, $Date: 2005/10/18 22:20:15 $
 */
public final class Dollar implements Arg, Variable, Mutable, Indexed {

	private final Slash _slash;

	private final String _name;

	private int _index = 0;

	private boolean _hasMostGeneralSlash = false;

	public Dollar(String name) {
		this(new Slash(), name);
	}

	public Dollar(Slash s, String name) {
		this(s, name, 0);
	}

	public Dollar(Slash s, String name, int id) {
		_slash = s;
		_name = name;
		_index = id;
		if (s.toString().equals("|.")) {
			_hasMostGeneralSlash = true;
		}
	}

	public String name() {
		return _name;
	}

	public int getIndex() {
		return _index;
	}

	public void setIndex(int uniqueIndex) {
		_index = uniqueIndex;
	}

	public Arg copy() {
		return new Dollar(_slash.copy(), _name, _index);
	}

	public void forall(CategoryFcn fcn) {
	}

	public Slash getSlash() {
		return _slash;
	}

	public boolean equals(Object o) {
		return (o instanceof Dollar && _index == ((Dollar) o).getIndex() && _slash
				.equals(((Dollar) o).getSlash()));
	}

	public int hashCode() {
		return toString().hashCode();
	}

	public boolean occurs(Variable v) {
		return (v instanceof Dollar && equals(v));
	}

	public Object fill(Substitution sub) throws UnifyFailure {
		Object value = sub.getValue(this);
		if (value == null) {
			return this;
		}
		if (value instanceof Dollar) {
			return value;
		}
		// nb: must do occurs check here, at least in part b/c ArgStack doesn't
		// quite implement Unifiable
		if (value instanceof Arg && !((Arg) value).occurs(this)) {
			return ((Arg) value).fill(sub);
		} else if (value instanceof ArgStack
				&& !((ArgStack) value).occurs(this)) {
			return ((ArgStack) value).fill(sub);
		} else {
			// System.out.println("Error in value for dollar: " + this +" = " +
			// value);
			throw new UnifyFailure();
		}
	}

	public void unifySlash(Slash s) throws UnifyFailure {
		_slash.unifyCheck(s);
	}

	public void unifyCheck(Object u) throws UnifyFailure {
	}

	public Object unify(Object u, Substitution sub) throws UnifyFailure {
		if (u instanceof ArgStack && !((ArgStack) u).occurs(this)) {
			((ArgStack) u).slashesUnify(_slash);
		} else if (u instanceof Arg && !((Arg) u).occurs(this)) {
			((Arg) u).unifySlash(_slash);
		} else {
			throw new UnifyFailure();
		}
		sub.makeSubstitution(this, u);
		return u;
	}

	public void deepMap(ModFcn mf) {
		mf.modify(this);
	}

	public String toString() {
		StringBuffer sb = new StringBuffer();
		if (!_hasMostGeneralSlash)
			sb.append(_slash.toString());
		sb.append('$').append(_name);// .append(_index);
		return sb.toString();
	}

	/**
	 * Returns the supertag for this dollar arg.
	 */
	public String getSupertag() {
		StringBuffer sb = new StringBuffer();
		if (!_hasMostGeneralSlash)
			sb.append(_slash.getSupertag());
		sb.append('$');
		return sb.toString();
	}

	/**
	 * Returns a TeX-formatted string representation for this dollar arg.
	 */
	public String toTeX() {
		StringBuffer sb = new StringBuffer();
		if (!_hasMostGeneralSlash)
			sb.append(_slash.toTeX());
		sb.append("\\$ \\subs{").append(_name).append("}");// .append(_index);
		return sb.toString();
	}

	/**
	 * Returns a hash code using the given map from vars to ints.
	 */
	public int hashCode(TObjectIntHashMap varMap) {
		int retval = _slash.hashCode(varMap);
		// see if this already in map
		if (varMap.containsKey(this)) {
			retval += varMap.get(this);
		}
		// otherwise add it
		else {
			int next = varMap.size() + 1;
			varMap.put(this, next);
			retval += next;
		}
		return retval;
	}

	/**
	 * Returns whether this dollar equals the given object up to variable names,
	 * using the given maps from vars to ints.
	 */
	public boolean equals(Object obj, TObjectIntHashMap varMap,
			TObjectIntHashMap varMap2) {
		if (obj.getClass() != this.getClass()) {
			return false;
		}
		Dollar d = (Dollar) obj;
		if (!_slash.equals(d._slash, varMap, varMap2)) {
			return false;
		}
		if (varMap.get(this) != varMap2.get(d))
			return false;
		return true;
	}
}
