/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package balt.corba.data.translation;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.omg.CORBA.Any;
import org.omg.CORBA.ORB;
import org.omg.CORBA.TCKind;
import org.omg.CORBA.TypeCode;

public class GenericSequenceTranslator<T> implements
        FrameworkDataTranslator<T>

{

    private Method m_extractMethod;
    private Method m_insertMethod;
    private TCKind m_kind;
    private Class<T> m_class;

    void init(Class _helperClass)
            throws FrameworkDataTranslatorException {
        try {
            // TODO could add more checking for handled type of helper!

            m_extractMethod = _helperClass.getMethod("extract",
                new Class[] {
                    Any.class
                });

            Method methods[] = _helperClass.getMethods();
            for (int i = 0; i < methods.length; i++) {
                if (methods[i].getName().equals("insert")) {
                    m_insertMethod = methods[i];
                }
            }

            if (m_insertMethod == null) {
                throw new FrameworkDataTranslatorException(
                    "unable to find \"insert\" method in: "
                        + _helperClass);
            }

            Method typeMethod = _helperClass.getMethod("type",
                new Class[0]);

            // System.out.println("gst helper: " + m_class);
            // System.out.println("gst helper: " + _helperClass);
            // System.out.println("gst helper: " + m_extractMethod);
            // System.out.println("gst helper: " + m_insertMethod);
            // System.out.println("gst helper: " + typeMethod);
            //            
            TypeCode tc = (TypeCode) typeMethod.invoke(null,
                new java.lang.Object[0]);

            m_kind = tc.kind();
            // System.out.println("gst helper: " + m_kind);
        }
        catch (SecurityException e) {
            throw new FrameworkDataTranslatorException(e);
        }
        catch (NoSuchMethodException e) {
            throw new FrameworkDataTranslatorException(e);
        }
        catch (IllegalArgumentException e) {
            throw new FrameworkDataTranslatorException(e);
        }
        catch (IllegalAccessException e) {
            throw new FrameworkDataTranslatorException(e);
        }
        catch (InvocationTargetException e) {
            throw new FrameworkDataTranslatorException(e);
        }
    }

    /**
     * 
     */
    public GenericSequenceTranslator(Class<T> _class, Class _helperClass) {
        try {
            m_class = _class;
            init(_helperClass);
        }
        catch (FrameworkDataTranslatorException e) {
            System.err.println("Translator not constructed because: "
                + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see sandbox.ReceiverTranslator#translate(T)
     */
    public Any translate(T _data)
            throws FrameworkDataTranslatorException {
        Any a = ORB.init().create_any();
        
//        System.err.println("translating: " +_data.getClass());
        
        try {
            m_insertMethod.invoke(null, new java.lang.Object[] {
                a, _data
            });
            return a;
        }
        catch (IllegalArgumentException e) {
            e.printStackTrace();
            throw new FrameworkDataTranslatorException(e);
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
            throw new FrameworkDataTranslatorException(e);
        }
        catch (InvocationTargetException e) {
            e.printStackTrace();
            throw new FrameworkDataTranslatorException(e);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see sandbox.ReceiverTranslator#translate(org.omg.CORBA.Any)
     */
    @SuppressWarnings("unchecked")
    public T translate(Any _data)
            throws FrameworkDataTranslatorException {
        try {
//             System.out.println("extract: " + m_extractMethod);
            return (T) m_extractMethod.invoke(null,
                new java.lang.Object[] {
                    _data
                });
        }
        catch (IllegalArgumentException e) {
            e.printStackTrace();
            throw new FrameworkDataTranslatorException(e);
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
            throw new FrameworkDataTranslatorException(e);
        }
        catch (InvocationTargetException e) {
            e.printStackTrace();
            throw new FrameworkDataTranslatorException(e);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see sandbox.ReceiverTranslator#getTCKind()
     */
    public TCKind getTCKind() {
        return m_kind;
    }

    /*
     * (non-Javadoc)
     * 
     * @see sandbox.FrameworkDataTranslator#getTransClass()
     */

    public Class<T> getTransClass() {
        return m_class;
    }

}
