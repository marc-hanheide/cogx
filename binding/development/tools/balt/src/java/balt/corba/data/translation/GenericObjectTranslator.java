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

public class GenericObjectTranslator<T> implements
        FrameworkDataTranslator<T> {

    private Method m_extractMethod;
    private Method m_insertMethod;
    private TCKind m_kind;
    private Class<T> m_class;

    private void init(Class<T> _class)
            throws FrameworkDataTranslatorException {
        ClassLoader loader = ClassLoader.getSystemClassLoader();
        String helperName = _class.getName() + "Helper";
        
        try {
            Class helperClass = loader.loadClass(helperName);
        
            m_extractMethod = helperClass.getMethod("extract",
                new Class[] {
                    Any.class
                });
            m_insertMethod = helperClass.getMethod("insert",
                new Class[] {
                    Any.class, _class
                });

            Method typeMethod = helperClass.getMethod("type",
                new Class[0]);

            TypeCode tc = (TypeCode) typeMethod.invoke(null,
                new Object[0]);
            m_kind = tc.kind();

        }
        catch (ClassNotFoundException e) {
            String classPath = System.getProperty("java.class.path");
            System.out.println("classpath is: " + classPath);
            throw new FrameworkDataTranslatorException(e);
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
    public GenericObjectTranslator(Class<T> _class) {
        try {
            m_class = _class;
            init(_class);
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

        try {
            m_insertMethod.invoke(null, new Object[] {
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
            return (T) m_extractMethod.invoke(null, new Object[] {
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
