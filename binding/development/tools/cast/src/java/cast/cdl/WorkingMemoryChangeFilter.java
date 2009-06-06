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

public final class WorkingMemoryChangeFilter implements java.lang.Cloneable, java.io.Serializable
{
    public WorkingMemoryOperation operation;

    public String src;

    public WorkingMemoryAddress address;

    public String type;

    public FilterRestriction restriction;

    public String origin;

    public WorkingMemoryChangeFilter()
    {
    }

    public WorkingMemoryChangeFilter(WorkingMemoryOperation operation, String src, WorkingMemoryAddress address, String type, FilterRestriction restriction, String origin)
    {
        this.operation = operation;
        this.src = src;
        this.address = address;
        this.type = type;
        this.restriction = restriction;
        this.origin = origin;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        WorkingMemoryChangeFilter _r = null;
        try
        {
            _r = (WorkingMemoryChangeFilter)rhs;
        }
        catch(ClassCastException ex)
        {
        }

        if(_r != null)
        {
            if(operation != _r.operation && operation != null && !operation.equals(_r.operation))
            {
                return false;
            }
            if(src != _r.src && src != null && !src.equals(_r.src))
            {
                return false;
            }
            if(address != _r.address && address != null && !address.equals(_r.address))
            {
                return false;
            }
            if(type != _r.type && type != null && !type.equals(_r.type))
            {
                return false;
            }
            if(restriction != _r.restriction && restriction != null && !restriction.equals(_r.restriction))
            {
                return false;
            }
            if(origin != _r.origin && origin != null && !origin.equals(_r.origin))
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
        if(operation != null)
        {
            __h = 5 * __h + operation.hashCode();
        }
        if(src != null)
        {
            __h = 5 * __h + src.hashCode();
        }
        if(address != null)
        {
            __h = 5 * __h + address.hashCode();
        }
        if(type != null)
        {
            __h = 5 * __h + type.hashCode();
        }
        if(restriction != null)
        {
            __h = 5 * __h + restriction.hashCode();
        }
        if(origin != null)
        {
            __h = 5 * __h + origin.hashCode();
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
        operation.__write(__os);
        __os.writeString(src);
        address.__write(__os);
        __os.writeString(type);
        restriction.__write(__os);
        __os.writeString(origin);
    }

    public void
    __read(IceInternal.BasicStream __is)
    {
        operation = WorkingMemoryOperation.__read(__is);
        src = __is.readString();
        address = new WorkingMemoryAddress();
        address.__read(__is);
        type = __is.readString();
        restriction = FilterRestriction.__read(__is);
        origin = __is.readString();
    }
}
