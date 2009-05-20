///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2002-4 Jason Baldridge, Gunes Erkan and 
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
package opennlp.ccg.unify;

import opennlp.ccg.grammar.*;

/**
 * A class for variables which can stand for any feature.
 *
 * @author      Jason Baldridge
 * @author      Gunes Erkan
 * @author      Michael White
 * @version     $Revision: 1.7 $, $Date: 2005/10/20 17:30:30 $
 **/
public class GFeatVar implements Variable, Indexed, Mutable {
    
    protected final String _name;
    protected int _index;
    protected int _hashCode;
    protected SimpleType type;
    
    public GFeatVar(String name) {
        this(name, 0, null);
    }

    public GFeatVar(String name, SimpleType st) {
        this(name, 0, st);
    }

    protected GFeatVar(String name, int index, SimpleType st) {
        _name = name;
        _index = index;
        type = (st != null) ? st : Grammar.theGrammar.types.getSimpleType(Types.TOP_TYPE);
        _hashCode = _name.hashCode() + _index + type.getIndex();
    }
    
    public String name() {
        return _name;
    }

    public Object copy() {
        return new GFeatVar(_name, _index, type);
    }
    
    public void deepMap(ModFcn mf) {
        mf.modify(this);
    }
    
    public int getIndex() {
        return _index;
    }

    public SimpleType getType() {
        return type;
    }

    public void setIndex(int index) {
        _hashCode += index - _index;
        _index = index;
    }

    public boolean occurs(Variable var) {
        return equals(var);
    }

    public int hashCode() {
        return _hashCode;
    }
    
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof GFeatVar)) return false;
        GFeatVar var = (GFeatVar) o;
        return _index == var._index && _name.equals(var._name) && type.equals(var.type);
    }

    public void unifyCheck(Object o) throws UnifyFailure {}
    
    public Object unify(Object u, Substitution sub) throws UnifyFailure {
        if (equals(u)) {
            return this;
        }
        else if (u instanceof SimpleType) {
            SimpleType st1 = getType();
            SimpleType st2 = (SimpleType)u;
            return sub.makeSubstitution(this, st2.unify(st1, sub));
        }
        else if (u instanceof GFeatVar) {
            GFeatVar var = (GFeatVar) u;
            if (var.occurs(this)) throw new UnifyFailure();
            SimpleType st1 = getType();
            SimpleType st2 = var.getType();
            SimpleType st3 = (SimpleType) st2.unify(st1, sub);
            // substitute var with most specific type
            if (st3.equals(st2)) return sub.makeSubstitution(this, var);
            else if (st3.equals(st1)) return sub.makeSubstitution(var, this);
            else {
                // need a new var with intersection type
                GFeatVar var3 = new GFeatVar(_name, UnifyControl.getUniqueVarIndex(), st3);
                sub.makeSubstitution(var, var3);
                return sub.makeSubstitution(this, var3);
            }
        }
        else {
            return sub.makeSubstitution(this, u);
        }
    }

    public Object fill(Substitution sub) throws UnifyFailure {
        Object val = sub.getValue(this);
        if (val != null) {
            return val;
        } else {
            return this;
        }
    }


    public String toString() { 
        String retval = _name;
        if (!type.getName().equals(Types.TOP_TYPE)) retval += ":" + type.getName();
        return retval;
    }
}
