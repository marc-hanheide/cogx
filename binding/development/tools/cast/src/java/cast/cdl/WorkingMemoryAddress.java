// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1

package cast.cdl;

public final class WorkingMemoryAddress implements java.lang.Cloneable, java.io.Serializable
{
    public String id;

    public String subarchitecture;

    public WorkingMemoryAddress()
    {
    }

    public WorkingMemoryAddress(String id, String subarchitecture)
    {
        this.id = id;
        this.subarchitecture = subarchitecture;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        WorkingMemoryAddress _r = null;
        try
        {
            _r = (WorkingMemoryAddress)rhs;
        }
        catch(ClassCastException ex)
        {
        }

        if(_r != null)
        {
            if(id != _r.id && id != null && !id.equals(_r.id))
            {
                return false;
            }
            if(subarchitecture != _r.subarchitecture && subarchitecture != null && !subarchitecture.equals(_r.subarchitecture))
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
        if(id != null)
        {
            __h = 5 * __h + id.hashCode();
        }
        if(subarchitecture != null)
        {
            __h = 5 * __h + subarchitecture.hashCode();
        }
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
        __os.writeString(id);
        __os.writeString(subarchitecture);
    }

    public void
    __read(IceInternal.BasicStream __is)
    {
        id = __is.readString();
        subarchitecture = __is.readString();
    }
}
