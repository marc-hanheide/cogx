///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2002 Jason Baldridge
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

/**
 * A class for variables which can stand for slash modalities.
 *
 * @author      Jason Baldridge
 * @version     $Revision: 1.2 $, $Date: 2004/05/01 10:40:04 $
 **/
public class VarModality implements Variable, Indexed, Mutable, Modality {
    
    protected final String _name;
    protected int _index;
    protected int _hashCode;
    
    private static int UNIQUE_STAMP = 0;
    
    public VarModality() {
        this("VM"+UNIQUE_STAMP++);
    }
    
    public VarModality(String name) {
        this(name, 0);
    }

    protected VarModality(String name, int index) {
        _name = name;
        _index = index;
        _hashCode = _name.hashCode() + _index;
    }
    
    public String name() {
        return _name;
    }

    public Object copy() {
        return new VarModality(_name, _index);
    }
    
    public void deepMap(ModFcn mf) {
        mf.modify(this);
    }
    
    public int getIndex() {
        return _index;
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
        if (o instanceof VarModality
            && _hashCode == ((VarModality)o)._hashCode) {
            //&& _index == ((VarModality)o)._index
            //&& _name.equals(((VarModality)o)._name)) {
            return true;
        } else {
            return false;
        }
    }

    public void unifyCheck(Object o) throws UnifyFailure {
        if (!(o instanceof SlashMode || o instanceof VarModality)) {
            throw new UnifyFailure();
        }
    }
    
    public Object unify(Object u, Substitution sub) throws UnifyFailure {
        if (u instanceof SlashMode) {
            return sub.makeSubstitution(this, u);    
        } else if (u instanceof VarModality) {
            VarModality var2 = (VarModality)u;
            Variable $var = new VarModality(_name+var2._name,
                            UnifyControl.getUniqueVarIndex());
            
            sub.makeSubstitution(this, $var);
            sub.makeSubstitution(var2, $var);
            return $var;
        } else {
            throw new UnifyFailure();
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

    public byte getDirection() {
        return Slash.B;
    }
    
    public String toString(byte dir) { 
        return toString();
    }

    public String toString() { 
        return _name;
    }
    
    public String toTeX(byte dir) {    
        return toTeX();
    }

    public String toTeX() {    
        return  _name;
    }
}
