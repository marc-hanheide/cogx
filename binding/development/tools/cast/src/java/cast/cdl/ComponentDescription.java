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

public final class ComponentDescription implements java.lang.Cloneable, java.io.Serializable
{
    public String componentName;

    public String className;

    public ComponentLanguage language;

    public String hostName;

    public java.util.Map<java.lang.String, java.lang.String> configuration;

    public ComponentDescription()
    {
    }

    public ComponentDescription(String componentName, String className, ComponentLanguage language, String hostName, java.util.Map<java.lang.String, java.lang.String> configuration)
    {
        this.componentName = componentName;
        this.className = className;
        this.language = language;
        this.hostName = hostName;
        this.configuration = configuration;
    }

    public boolean
    equals(java.lang.Object rhs)
    {
        if(this == rhs)
        {
            return true;
        }
        ComponentDescription _r = null;
        try
        {
            _r = (ComponentDescription)rhs;
        }
        catch(ClassCastException ex)
        {
        }

        if(_r != null)
        {
            if(componentName != _r.componentName && componentName != null && !componentName.equals(_r.componentName))
            {
                return false;
            }
            if(className != _r.className && className != null && !className.equals(_r.className))
            {
                return false;
            }
            if(language != _r.language && language != null && !language.equals(_r.language))
            {
                return false;
            }
            if(hostName != _r.hostName && hostName != null && !hostName.equals(_r.hostName))
            {
                return false;
            }
            if(configuration != _r.configuration && configuration != null && !configuration.equals(_r.configuration))
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
        if(componentName != null)
        {
            __h = 5 * __h + componentName.hashCode();
        }
        if(className != null)
        {
            __h = 5 * __h + className.hashCode();
        }
        if(language != null)
        {
            __h = 5 * __h + language.hashCode();
        }
        if(hostName != null)
        {
            __h = 5 * __h + hostName.hashCode();
        }
        if(configuration != null)
        {
            __h = 5 * __h + configuration.hashCode();
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
        __os.writeString(componentName);
        __os.writeString(className);
        language.__write(__os);
        __os.writeString(hostName);
        StringMapHelper.write(__os, configuration);
    }

    public void
    __read(IceInternal.BasicStream __is)
    {
        componentName = __is.readString();
        className = __is.readString();
        language = ComponentLanguage.__read(__is);
        hostName = __is.readString();
        configuration = StringMapHelper.read(__is);
    }
}
