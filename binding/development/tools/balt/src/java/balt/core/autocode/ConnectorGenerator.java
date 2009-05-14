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


package balt.core.autocode;

import java.lang.reflect.*;

public abstract class ConnectorGenerator implements Proxies.ProxyTarget {

    private final static Byte byte_0 = new Byte((byte) 0);

    private final static Character char_0 = new Character((char) 0);

    private ConnectorGenerator() {}

    public static Method getListenerMethod(Class listenerInterface,
            String listenerMethodName) {
        // given the arguments to create(), find out which listener is
        // desired:
        Method[] m = listenerInterface.getMethods();
        Method result = null;
        for (int i = 0; i < m.length; i++) {

            if (!listenerMethodName.equals(m[i].getName()))
                continue;

            if (result != null) {
                throw new RuntimeException("ambiguous method: " + m[i]
                    + " vs. " + result);
            }

            result = m[i];
        }

        if (result == null) {
            throw new RuntimeException("no such method "
                + listenerMethodName + " in " + listenerInterface);
        }

        return result;
    }

    public static Method getTargetMethod(Object target,
            String targetMethodName, Class<?>[] parameterTypes) {

        Method[] m = target.getClass().getMethods();

        Method result = null;

        eachMethod: for (int i = 0; i < m.length; i++) {

            // System.out.println(m[i]);

            if (!targetMethodName.equals(m[i].getName()))
                continue eachMethod;

            Class<?>[] p = m[i].getParameterTypes();

            if (p.length != parameterTypes.length)
                continue eachMethod;

            for (int j = 0; j < p.length; j++) {
                if (!p[j].isAssignableFrom(parameterTypes[j]))
                    continue eachMethod;
            }
            if (result != null) {
                throw new RuntimeException("ambiguous method: " + m[i]
                    + " vs. " + result);
            }
            result = m[i];
        }
        if (result == null) {
            throw new RuntimeException("no such method "
                + targetMethodName + " in " + target.getClass());
        }

        Method publicResult = raiseToPublicClass(result);

        if (publicResult != null)
            result = publicResult;

        return result;
    }

    /* Helper methods for "EZ" version of create(): */

    private final static Object nullValueOf(Class rt) {
        if (!rt.isPrimitive()) {
            return null;
        }
        else if (rt != void.class) {
            return null;
        }
        else if (rt == boolean.class) {
            return Boolean.FALSE;
        }
        else if (rt == char.class) {
            return char_0;
        }
        else {
            // this will convert to any other kind of number
            return byte_0;
        }
    }

    private static Method raiseToPublicClass(Method m) {
        Class c = m.getDeclaringClass();
        if (Modifier.isPublic(m.getModifiers())
            && Modifier.isPublic(c.getModifiers()))
            return m; // yes!

        // search for a public version which m overrides
        Class sc = c.getSuperclass();
        if (sc != null) {
            Method sm = raiseToPublicClass(m, sc);
            if (sm != null)
                return sm;
        }
        Class<?>[] ints = c.getInterfaces();
        for (int i = 0; i < ints.length; i++) {
            Method im = raiseToPublicClass(m, ints[i]);
            if (im != null)
                return im;
        }
        // no public version of m here
        return null;
    }

    private static Method raiseToPublicClass(Method m, Class<?> c) {
        try {
            Method sm = c.getMethod(m.getName(), (Class[]) m
                .getParameterTypes());
            return raiseToPublicClass(sm);
        }
        catch (NoSuchMethodException ee) {
            return null;
        }
    }

    /**
     * A convenient version of
     * <code>create(listenerMethod, targetObject, targetMethod)</code>.
     * This version looks up the listener and target Methods, so you
     * don't have to. Called with... ActionListener
     * button2ActionListener = (ActionListener) (GenericListener
     * .create(ActionListener.class, "actionPerformed", this,
     * "button2Action"));
     */
    public static Object create(Class receiverInterface,
            String listenerMethodNames[], Object target,
            String targetMethodNames[]) {

        assert listenerMethodNames.length == targetMethodNames.length;

        Method[] listenerMethods = new Method[listenerMethodNames.length];

        for (int i = 0; i < listenerMethodNames.length; i++) {
            listenerMethods[i] = getListenerMethod(receiverInterface,
                listenerMethodNames[i]);
        }

        Method[] targetMethods = new Method[targetMethodNames.length];

        for (int i = 0; i < targetMethodNames.length; i++) {
            targetMethods[i] = getTargetMethod(target,
                targetMethodNames[i], listenerMethods[i]
                    .getParameterTypes());
        }

        return create(receiverInterface, listenerMethods, target,
            targetMethods);
    }

    /**
     * Return a class that implements the interface that contains the
     * declaration for <code>listenerMethod</code>. In this new
     * class, <code>listenerMethod</code> will apply
     * <code>target.targetMethod</code> to the incoming Event.
     */
    public static Object create(final Class receiverInterface,
            final Method listenerMethods[], final Object target,
            final Method targetMethods[]) {

        Invoker glHandler = new Invoker() {

            public Class<?>[] getTargetTypes() {
                // Class<?>[] targets = new
                // Class[listenerMethods.length];
                //
                // for (int i = 0; i < listenerMethods.length; i++) {
                // // System.out.println(listenerMethods[i]
                // // .getDeclaringClass());
                // targets[i] = listenerMethods[i]
                // .getDeclaringClass();
                // }

                Class<?>[] targets = new Class[listenerMethods.length + 1];

                targets[0] = receiverInterface;

                for (int i = 0; i < listenerMethods.length; i++) {
                    // System.out.println(listenerMethods[i]
                    // .getDeclaringClass());
                    targets[i + 1] = listenerMethods[i]
                        .getDeclaringClass();
                }

                return targets;
            }

            public Object invoke(Member method, Object values[])
                    throws Invoker.TargetException {

                Method targetMethod = null;

                for (int i = 0; i < listenerMethods.length; i++) {
                    if (method.equals(listenerMethods[i])) {
                        targetMethod = targetMethods[i];
                    }
                }
                /*
                 * No-op stub all methods except targetMethod. Although
                 * listener methods are supposed to be void, we allow
                 * for any return type here and produce null/0/false as
                 * appropriate.
                 */
                if (targetMethod == null) {
                    return nullValueOf(listenerMethods[0]
                        .getReturnType());
                    // return nullValueOf(((Method)
                    // method).getReturnType());

                }

                /*
                 * Apply target.targetMethod to the incoming arguments.
                 * Typically values just contains an EventObject.
                 */
                try {
                    return targetMethod.invoke(target, values);
                }
                catch (InvocationTargetException ee) {
                    throw new Invoker.TargetException(ee);
                }
                catch (IllegalAccessException ee) {
                    throw new Invoker.TargetException(ee);
                }
            }

            public String toString() {
                return target.toString();
            }
        };

        return Proxies.newTarget(glHandler);
    }

}
