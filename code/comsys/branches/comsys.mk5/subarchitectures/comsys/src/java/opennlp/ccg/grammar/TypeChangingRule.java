///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-4 Jason Baldridge and University of Edinburgh (Michael White)
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

package opennlp.ccg.grammar;

import opennlp.ccg.unify.*;
import opennlp.ccg.synsem.*;

import java.util.*;

/**
 * A CCG unary type changing rule.
 *
 * @author      Jason Baldridge
 * @author      Michael White
 * @version $Revision: 1.7 $, $Date: 2005/10/18 22:20:15 $
 **/
public class TypeChangingRule extends AbstractRule {

    /** The argument category. */   
    protected Category _arg; 

    /** The result category. */
    protected Category _result; 
    
    /** The first elementary predication in the result LF (or null), before sorting. */
    protected LF _firstEP;


    /** Constructor. */
    public TypeChangingRule(Category arg, Category result, String name, LF firstEP) {
        _arg = arg; _result = result; _name = name; _firstEP = firstEP;
    }


    /** Returns 1. */
    public int arity() { return 1; }
    
    /** Returns the arg. */
    public Category getArg() { return _arg; }
    
    /** Returns the result. */
    public Category getResult() { return _result; }

    /** Returns the first elementary predication in the result LF (or null), before sorting. */
    public LF getFirstEP() { return _firstEP; }


    /** Applies this rule to the given inputs. */
    public List<Category> applyRule(Category[] inputs) throws UnifyFailure {
        // check arity
        if (inputs.length != 1) {
            throw new UnifyFailure();
        }
        return apply(inputs[0]);
    }

    /** Applies this rule to the given input. */
    protected List<Category> apply(Category input) throws UnifyFailure {

        // unify quick check
        _arg.unifyCheck(input);
        
        // copy arg and result
        Category arg = _arg.copy();
        Category result = _result.copy();
        
        // make variables unique
        UnifyControl.reindex(result, arg);

        // unify
        Substitution sub = new GSubstitution();
        GUnifier.unify(input, arg, sub);
        ((GSubstitution)sub).condense();

        // fill in result
        Category $result = (Category)result.fill(sub);
        appendLFs(input, result, $result, sub);

        // return
        List<Category> results = new ArrayList<Category>(1);
        results.add($result);
        return results;
    }
    
    
    /** Returns 'name: arg => result'. */
    public String toString() {
        StringBuffer sb = new StringBuffer();
        sb.append(_name).append(": ");
        sb.append(_arg).append(' ');
        sb.append("=> ").append(_result);
        return sb.toString();
    }
}

