// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package de.dfki.lt.tr.beliefs.slice.distribs;

public final class FormulaProbPair implements java.lang.Cloneable, java.io.Serializable
{
    public de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula val;

    public float prob;

    public FormulaProbPair()
    {
    }

    public FormulaProbPair(de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula val, float prob)
    {
        this.val = val;
        this.prob = prob;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        FormulaProbPair _r = null;
        try
        {
            _r = (FormulaProbPair)rhs;
        }
        catch(ClassCastException ex)
        {
        }

        if(_r != null)
        {
            if(val != _r.val && val != null && !val.equals(_r.val))
            {
                return false;
            }
            if(prob != _r.prob)
            {
                return false;
            }

            return true;
        }

        return false;
    }

    public int
    hashCode()
    {
        int __h = 0;
        if(val != null)
        {
            __h = 5 * __h + val.hashCode();
        }
        __h = 5 * __h + java.lang.Float.floatToIntBits(prob);
        return __h;
    }

    public java.lang.Object
    clone()
    {
        java.lang.Object o = null;
        try
        {
            o = super.clone();
        }
        catch(CloneNotSupportedException ex)
        {
            assert false; // impossible
        }
        return o;
    }

    public void
    __write(IceInternal.BasicStream __os)
    {
        __os.writeObject(val);
        __os.writeFloat(prob);
    }

    private class Patcher implements IceInternal.Patcher
    {
        public void
        patch(Ice.Object v)
        {
            try
            {
                val = (de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula)v;
            }
            catch(ClassCastException ex)
            {
                IceInternal.Ex.throwUOE(type(), v.ice_id());
            }
        }

        public String
        type()
        {
            return "::de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula";
        }
    }

    public void
    __read(IceInternal.BasicStream __is)
    {
        __is.readObject(new Patcher());
        prob = __is.readFloat();
    }
}
