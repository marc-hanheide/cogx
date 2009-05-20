///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2002-5 Jason Baldridge, Gann Bierner and
//                      University of Edinburgh (Michael White)
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
import org.jdom.Element;

/**
 * A categorial slash which has an optional mode associated with it.
 * 
 * @author Jason Baldridge
 * @author Gann Bierner
 * @author Michael White
 * @version $Revision: 1.6 $, $Date: 2005/10/18 22:20:15 $
 */
public final class Slash implements Unifiable, Mutable {

	public static final byte L = 0;

	public static final byte B = 1;

	public static final byte R = 2;

	public static final byte INERT_OR_ACTIVE = 0;

	public static final byte ACTIVE = 1;

	public static final byte INERT = 2;

	private final byte _dir;

	private final Modality _modality;

	private byte _ability = INERT_OR_ACTIVE;

	public Slash(Element el) {
		String d = el.getAttributeValue("dir");
		if (d == null)
			d = el.getAttributeValue("d");
		if (d == null) {
			d = "|";
		}
		_dir = encode(d.charAt(0));

		String m = el.getAttributeValue("mode");
		if (m == null)
			m = el.getAttributeValue("m");
		if (m != null) {
			_modality = new SlashMode(m);
		} else {
			String vm = el.getAttributeValue("varmodality");
			if (vm == null)
				vm = el.getAttributeValue("varModality");
			if (vm != null) {
				_modality = new VarModality(vm);
			} else {
				_modality = new SlashMode();
			}
		}

		String ability = el.getAttributeValue("ability");
		if (null != ability) {
			setAbility(ability);
		}
	}

	public Slash() {
		this('|');
	}

	public Slash(char sd) {
		_dir = encode(sd);
		_modality = new SlashMode();
	}

	public Slash(char sd, String md) {
		_dir = encode(sd);
		_modality = new SlashMode(md);
	}

	public Slash(char sd, Modality md) {
		_dir = encode(sd);
		_modality = md;
	}

	private Slash(byte d, Modality m, byte a) {
		_dir = d;
		_modality = m;
		_ability = a;
	}

	public Slash copy() {
		return new Slash(_dir, (Modality) _modality.copy(), _ability);
	}

	public boolean occurs(Variable v) {
		return _modality.occurs(v);
	}

	public void deepMap(ModFcn mf) {
		mf.modify(this);
	}

	public boolean isActive() {
		return _ability == ACTIVE || _ability == INERT_OR_ACTIVE;
	}

	public boolean setAbility(String ability) {
		byte newAbility;
		if (ability.equals("inert")) {
			newAbility = INERT;
		} else if (ability.equals("active")) {
			newAbility = ACTIVE;
		} else {
			newAbility = INERT_OR_ACTIVE;
		}
		if (abilitiesMatch(_ability, newAbility)) {
			_ability = newAbility;
			return true;
		} else {
			return false;
		}
	}

	public void unifyCheck(Object u) throws UnifyFailure {
		if (u instanceof Slash) {
			if (!abilitiesMatch(_ability, ((Slash) u)._ability)) {
				throw new UnifyFailure();
			}
			if (!directionsMatch(_dir, ((Slash) u)._dir)) {
				throw new UnifyFailure();
			}
			_modality.unifyCheck(((Slash) u)._modality);
		} else {
			throw new UnifyFailure();
		}
	}

	public Object unify(Object u, Substitution sub) throws UnifyFailure {

		if (u instanceof Slash) {
			Slash s2 = (Slash) u;
			byte newAbility = _ability;
			if (_ability == INERT_OR_ACTIVE) {
				newAbility = s2._ability;
			} else if (s2._ability == INERT_OR_ACTIVE) {
				newAbility = _ability;
			} else if (_ability != s2._ability) {
				throw new UnifyFailure();
			}

			byte newDir = _dir;
			if (_dir == B) {
				newDir = s2._dir;
			} else if (s2._dir == B) {
				newDir = _dir;
			} else if (_dir != s2._dir) {
				throw new UnifyFailure();
			}

			Modality newModality = (Modality) _modality.unify(
					((Slash) u)._modality, sub);
			return new Slash(newDir, newModality, newAbility);
		} else {
			throw new UnifyFailure();
		}

	}

	public Object fill(Substitution sub) throws UnifyFailure {
		return new Slash(_dir, (Modality) _modality.fill(sub), _ability);
	}

	public boolean equals(Slash s) {
		return directionsMatch(_dir, s._dir);
	}

	public boolean sameDirAsModality() {
		return directionsMatch(_dir, _modality.getDirection());
	}

	private static byte encode(char sd) {
		switch (sd) {
		case '/':
			return R;
		case '\\':
			return L;
		default:
			return B;
		}
	}

	public static boolean directionsMatch(byte s1, byte s2) {
		if (s1 == B || s2 == B) {
			return true;
		} else {
			return s1 == s2;
		}
	}

	private static boolean abilitiesMatch(byte ab1, byte ab2) {
		if (ab1 == INERT_OR_ACTIVE || ab2 == INERT_OR_ACTIVE) {
			return true;
		} else {
			return ab1 == ab2;
		}
	}

	/**
	 * Returns a hash code using the given map from vars to ints.
	 */
	public int hashCode(TObjectIntHashMap varMap) {
		return toString().hashCode() + _ability;
	}

	/**
	 * Returns whether this slash equals the given object up to variable names,
	 * using the given maps from vars to ints.
	 */
	public boolean equals(Object obj, TObjectIntHashMap varMap,
			TObjectIntHashMap varMap2) {
		if (obj.getClass() != this.getClass()) {
			return false;
		}
		Slash s = (Slash) obj;
		return _ability == s._ability && toString().equals(s.toString());
	}

	// string for showing ability
	private String abilityStr() {
		if (_ability == ACTIVE)
			return "@";
		else if (_ability == INERT)
			return "!";
		else
			return "";
	}

	public String toString() {
		switch (_dir) {
		case R:
			return "/" + abilityStr() + _modality.toString(R);
		case L:
			return "\\" + abilityStr() + _modality.toString(L);
		default:
			return "|" + abilityStr() + _modality;
		}
	}

	/**
	 * Returns the supertag for this slash.
	 */
	public String getSupertag() {
		switch (_dir) {
		case R:
			return "/";
		case L:
			return "\\";
		default:
			return "|";
		}
	}

	/**
	 * Returns a TeX-formatted string representation for this slash.
	 */
	public String toTeX() {
		StringBuffer sb = new StringBuffer();
		String sup = "\\sups";
		String sub = "\\subs";
		String modTeX = null;
		switch (_dir) {
		case R:
			sb.append("/ ");
			sup = "\\supsb";
			sub = "\\subsa";
			modTeX = _modality.toTeX(R);
			break;
		case L:
			sb.append("\\bs ");
			sub = "\\subsb";
			sup = "\\supsa";
			modTeX = _modality.toTeX(L);
			break;
		default:
			sb.append("| ");
			sub = "\\subs";
			sup = "\\sups";
			modTeX = _modality.toTeX();
			break;
		}

		if (_ability == ACTIVE)
			sb.append(sup).append("{").append("+").append("} ");
		if (_ability == INERT)
			sb.append(sup).append("{").append("-").append("} ");
		if ((modTeX != "") && ((_ability == ACTIVE) || (_ability == ACTIVE))) {
			if ((_dir == R))
				sb.append("\\hspace{-1.45mm} ");
			else if (_dir == L)
				sb.append("\\hspace{-0.50mm} ");
		}
		if (modTeX != "")
			sb.append(sub).append("{").append(modTeX).append("} ");
		return sb.toString();
	}
}
