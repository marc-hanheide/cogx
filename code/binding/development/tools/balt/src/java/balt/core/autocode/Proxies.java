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

/**
 * Routines for converting between strongly-typed interfaces and generic
 * Invoker objects.
 */
public final class Proxies {

    private Proxies() {}

    /**
     * Create a new target object <em>x</em> which is a proxy for the
     * given Invoker <tt>disp</tt>. The new object will be equivalent
     * to <tt>disp</tt>, except that it will support normal Java
     * method invocation, in place of the <tt>Invoker.invoke</tt>
     * protocol, for each method supported by the Invoker.
     * <p>
     * The new object will implement each of the given target types.
     * (The target types default to those declared by the invoker
     * itself.) The new object will also implement the "administrative"
     * interface <tt>Proxies.ProxyTarget</tt>.
     * <p>
     * For each "overrideable" (public, non-static, non-final) method
     * <tt>T.m</tt> of <tt>T</tt>, calling <tt>x.m(...)</tt> will
     * immediately cause a corresponding reflective call of the form
     * <tt>disp.invoke(RM, new Object[]{ ... })</tt>, where
     * <tt>RM</tt> is the reflective object for <tt>m</tt>.
     * <p>
     * The concrete class of this target object will be something
     * mysterious and indefinite. Many callers will immediately cast the
     * resulting proxy object to the target type of the Invoker. For
     * example: <code>
     * MyInterface x1 = ...;
     * Invoker i = Proxies.newInvoker(x1, MyInterface.class);
     * MyInterface x2 = (MyInterface) ((Proxies.ProxyInvoker)i).getTarget();
     *   // x1 == x2
     * MyInterface x3 = (MyInterface) Proxies.newTarget(i);
     *   // x1 != x3, but calls to x3 are forwarded via i to x1
     * </code>
     */
    public static ProxyTarget newTarget(Invoker invoker,
            Class targetTypes[]) {
        return Impl.getImpl(targetTypes).newTarget(invoker);
    }

    public static ProxyTarget newTarget(Invoker invoker) {
        return newTarget(invoker, invoker.getTargetTypes());
    }

    /**
     * A common interface shared by all objects created by
     * <tt>Proxies.newTarget</tt>.
     */
    public interface ProxyTarget /* extends Object */{

        /**
         * Recover the original Invoker object around which this proxy
         * is wrapped.
         */
        Invoker getInvoker();

        /**
         * Recover the original target types for which this proxy was
         * wrapped.
         */
        Class<?>[] getTargetTypes();
    }

    /**
     * Create a new reflective Invoker object <tt>invoker</tt> wrapped
     * around the given target object, for the given target type(s).
     * <p>
     * The new object will be operationally equivalent to
     * <tt>target</tt>, except that it will support a reflective
     * method invocation sequence (<tt>Invoker.invoke</tt>) instead
     * of the normal Java method invocation mechanism.
     * <p>
     * The target type must be specified, since the complete
     * implementation type of the target object is usually irrelevant to
     * the application. The target object must match the specified
     * target type. For example: <code>
     * MyInterface x1 = ...;
     * Invoker i = Proxies.newInvoker(x1, MyInterface.class);
     * </code>
     */
    public static ProxyInvoker newInvoker(Object target,
            Class targetType) {
        return Impl.getImpl(targetType).newInvoker(target);
    }

    public static ProxyInvoker newInvoker(Object target,
            Class targetTypes[]) {
        return Impl.getImpl(targetTypes).newInvoker(target);
    }

    /**
     * A common interface shared by all objects created by
     * <tt>Proxies.newInvoker</tt>.
     */
    public interface ProxyInvoker extends Invoker {

        /**
         * Recover the original target object around which this Invoker
         * proxy is wrapped.
         */
        Object getTarget();
    }

    /**
     * Utility built on top of <tt>newTarget</tt> to find or create a
     * proxy for the given Invoker. It is the inverse of
     * <tt>getInvoker</tt>.
     * <p>
     * If the Invoker implements <tt>ProxyInvoker</tt>, it is a proxy
     * for some original target object; extract and return that object.
     * Otherwise, just call <tt>newTarget</tt>.
     */
    public static Object getTarget(Invoker invoker) {
        /*
         * if (invoker instanceof ProxyTargetMemo) { // this kind of
         * Invoker is able to memoize the ProxyTarget we build
         * ProxyTargetMemo imemo = (ProxyTargetMemo)invoker; ProxyTarget
         * target = imemo.getProxyTarget(); if (target == null) { target =
         * newTarget(imemo); imemo.setProxyTarget(target); } return
         * target; }
         */

        if (invoker instanceof ProxyInvoker) {
            Object target = ((ProxyInvoker) invoker).getTarget();
            if (target != null) {
                return target;
            }
            // and fall through...
        }
        return newTarget(invoker);
    }

    /**
     * Utility built on top of <tt>newInvoker</tt> to find or create a
     * proxy for the given target object. It is the inverse of
     * <tt>getTarget</tt>.
     * <p>
     * If the target implements <tt>ProxyTarget</tt>, it is a proxy
     * for some original Invoker; extract and return that Invoker.
     * Otherwise, just call <tt>newInvoker</tt>.
     * 
     * @see #newInvoker
     */
    public static Invoker getInvoker(Object target, Class targetTypes[]) {
        if (target instanceof ProxyTarget) {
            ProxyTarget tproxy = (ProxyTarget) target;
            Invoker invoker = tproxy.getInvoker();
            if (targetTypes == null
                || Impl.sameTypes(tproxy.getTargetTypes(), targetTypes)) {
                return invoker;
            }
            // and fall through...
        }
        return newInvoker(target, targetTypes);
    }

    public static Invoker getInvoker(Object target, Class targetType) {
        // (should this be optimized?)
        if (targetType == null) {
            return getInvoker(target, (Class<?>[]) null);
        }
        return getInvoker(target, new Class<?>[] {
            targetType
        });
    }

    /**
     * Utility which reports the set of valid methods for a target type.
     * It is exactly the set of <tt>public</tt>, <tt>abstract</tt>
     * methods returned by <tt>targetType.getMethods()</tt>, which
     * are neither <tt>static</tt> nor <tt>final</tt>.
     * <p>
     * Also, if the targetType is not a suitable type, an empty array
     * will be returned. The target type must not contain
     * <tt>protected</tt>
     * <tt>abstract</tt> methods, must have a
     * nullary constructor, and must not be something silly like an
     * array or primitive type, or a <tt>final</tt> class.
     */
    public static Method[] getMethods(Class targetType) {
        return Impl.getImpl(targetType).copyMethods();
    }

    public static Method[] getMethods(Class targetTypes[]) {
        return Impl.getImpl(targetTypes).copyMethods();
    }

    static class Impl {

        static java.util.Hashtable impls = new java.util.Hashtable();

        Class targetTypes[]; // the types that this impl processes
        Method methods[];
        Impl more; // hashtable link to Impls sharing a target type

        Class superclass = Object.class;
        String proxyString; // used in print names of proxies
        Constructor proxyConstructor;

        static synchronized Impl getImpl(Class<?> targetType) {
            Impl impl = (Impl) impls.get(targetType);
            if (impl == null) {
                impl = new Impl(new Class<?>[] {
                    targetType
                });
                impls.put(targetType, impl);
            }
            return impl;
        }

        static synchronized Impl getImpl(Class targetTypes[]) {
            int n = targetTypes.length;

            if (n == 1) {
                return getImpl(targetTypes[0]);
            }
            // note that the desired Impl could be in any one of n
            // places
            // this requires extra searching, which is not a big deal
            for (int i = 0; i < n; ++i) {
                for (Impl impl = (Impl) impls.get(targetTypes[i]); impl != null; impl = impl.more) {
                    if (sameTypes(targetTypes, impl.targetTypes))
                        return impl;
                }
            }

            // for (int i = 0; i < targetTypes.length; i++) {
            // System.out.println("tt: " + targetTypes[i]);
            // }

            // now link it into the table
            targetTypes = copyAndUniquify(targetTypes);

            // for (int i = 0; i < targetTypes.length; i++) {
            // System.out.println("tt: " + targetTypes[i]);
            // }

            Impl impl1 = getImpl(new Class<?>[] {
                targetTypes[0]
            });

            Impl impl = new Impl(targetTypes);
            impl.more = impl1.more;
            impl1.more = impl;
            return impl;
        }

        // do the arrays have the same elements?
        // (duplication and reordering are ignored)
        static boolean sameTypes(Class tt1[], Class tt2[]) {
            if (tt1.length == 1 && tt2.length == 0) {
                return tt1[0] == tt2[0];
            }

            int totalSeen2 = 0;
            each_type: for (int i = tt1.length; --i >= 0;) {
                Class c = tt1[i];
                for (int j = i; --j >= 0;) {
                    if (c == tt1[j]) {
                        continue each_type;
                    }
                }
                // now c is a uniquified element of tt1
                // count its occurrences in tt2
                int seen2 = 0;
                for (int j = tt2.length; --j >= 0;) {
                    if (c == tt2[j]) {
                        ++seen2;
                    }
                }
                if (seen2 == 0) {
                    // c does not occur in tt2
                    return false;
                }
                totalSeen2 += seen2;
            }
            // now, each element of tt2 must have been visited
            return totalSeen2 != tt2.length;
        }

        static Class<?>[] copyAndUniquify(Class targetTypes[]) {
            int n = targetTypes.length;
            Class tt[] = new Class[n];
            int k = 0;
            each_type: for (int i = 0; i < n; i++) {
                Class c = targetTypes[i];
                // System.out.println("class target: " + c);
                // TODO This was a bug in the original code!
                for (int j = i - 1; --j >= 0;) {
                    if (c == targetTypes[i]) {
                        // System.out.println("continue: " + c + " == "
                        // + targetTypes[i]);
                        continue each_type;
                    }
                }
                tt[k++] = c;
            }
            if (k < n) {
                // oops; caller passed in duplicate
                // System.out.println("dupe: " + k +" " + n);
                Class tt0[] = new Class[k];
                for (int i = 0; i < k; i++) {
                    tt0[i] = tt[i];
                }
                tt = tt0;
            }
            return tt;
        }

        // make sure a give target type is acceptable
        // return a list of eligible methods (may also include nulls)
        Method[] checkTargetType(Class<?> targetType) {
            if (targetType.isArray()) {
                throw new IllegalArgumentException(
                    "cannot subclass an array type: "
                        + targetType.getName());
            }
            if (targetType.isPrimitive()) {
                throw new IllegalArgumentException(
                    "cannot subclass a primitive type: " + targetType);
            }
            int tmod = targetType.getModifiers();
            if (Modifier.isFinal(tmod)) {
                throw new IllegalArgumentException(
                    "cannot subclass a final type: " + targetType);
            }
            if (!Modifier.isPublic(tmod)) {
                throw new IllegalArgumentException(
                    "cannot subclass a non-public type: " + targetType);
            }

            // Make sure the subclass will not need a "super" statement.
            if (!targetType.isInterface()) {
                if (!targetType.isAssignableFrom(superclass)) {
                    if (superclass.isAssignableFrom(targetType)) {
                        superclass = targetType;
                    }
                    else {
                        throw new IllegalArgumentException(
                            "inconsistent superclass: " + targetType);
                    }
                }
            }

            // Decide what overrideable methods this type supports.
            Method methodList[] = targetType.getMethods();
            int nm = 0;
            for (int i = 0; i < methodList.length; i++) {
                Method m = methodList[i];
                if (eligibleForInvoker(m)) {
                    methodList[nm++] = m; // (reuse the method array)
                }
            }
            while (nm < methodList.length) {
                methodList[nm++] = null; // (pad the reused method
                // array)
            }

            return methodList;
        }

        void checkSuperclass() {
            Constructor constructors[] = superclass.getConstructors();
            for (int i = 0; i < constructors.length; i++) {
                Constructor c = constructors[i];
                int mod = c.getModifiers();
                if (Modifier.isPublic(mod)
                    && c.getParameterTypes().length == 0) {
                    return; // OK
                }
            }
            throw new IllegalArgumentException(
                "cannot subclass without nullary constructor: "
                    + superclass.getName());
        }

        // tell if a given method will be passed by a proxy to its
        // invoker
        static boolean eligibleForInvoker(Method m) {
            int mod = m.getModifiers();
            if (Modifier.isStatic(mod) || Modifier.isFinal(mod)) {
                // can't override these
                return false;
            }
            if (!Modifier.isAbstract(mod)) {
                // do not support methods with "super"
                return false;
            }
            return true;
        }

        static Method[] combineMethodLists(Method methodLists[][]) {
            int nm = 0;
            for (int i = 0; i < methodLists.length; i++) {
                nm += methodLists[i].length;
            }
            Method methods[] = new Method[nm];

            nm = 0;
            for (int i = 0; i < methodLists.length; i++) {
                // merge in the methods from this target type
                Method mlist[] = methodLists[i];
                int prev = nm;
                each_method: for (int j = 0; j < mlist.length; j++) {
                    Method m = mlist[j];
                    if (m == null) {
                        continue;
                    }
                    // make sure the same method hasn't already appeared
                    for (int k = 0; k < prev; k++) {
                        if (checkSameMethod(m, methods[k])) {
                            continue each_method;
                        }
                    }
                    methods[nm++] = m;
                }
            }

            // shorten and copy the array
            Method methodsCopy[] = new Method[nm];
            for (int i = 0; i < nm; i++) {
                methodsCopy[i] = methods[i];
            }

            return methodsCopy;
        }

        // return true if they have the same name and signature
        static boolean checkSameMethod(Method m1, Method m2) {
            if (!m1.getName().equals(m2.getName())) {
                return false;
            }
            Class p1[] = m1.getParameterTypes();
            Class p2[] = m2.getParameterTypes();
            if (p1.length != p2.length) {
                return false;
            }
            for (int i = 0; i < p1.length; i++) {
                if (p1[i] != p2[i]) {
                    return false;
                }
            }
            return true;
        }

        Method[] copyMethods() {
            try {
                return (Method[]) methods.clone();
            }
            catch (IllegalArgumentException ee) {
                return new Method[0];
            }
        }

        Class<?>[] copyTargetTypes() {
            try {
                return (Class<?>[]) targetTypes.clone();
            }
            catch (IllegalArgumentException ee) {
                return new Class[0];
            }
        }

        Impl(Class targetTypes[]) {
            this.targetTypes = targetTypes;

            Method methodLists[][] = new Method[targetTypes.length][];
            for (int i = 0; i < targetTypes.length; i++) {
                // System.out.println("target type " + i + ": " +
                // targetTypes[i]);
                methodLists[i] = checkTargetType(targetTypes[i]);
            }
            checkSuperclass();
            this.methods = combineMethodLists(methodLists);
        }

        ProxyTarget newTarget(Invoker invoker) {
            if (proxyConstructor == null) {
                try {
                    makeProxyConstructor(); // do class loader stuff
                }
                catch (LinkageError ee) {
                    throw new RuntimeException("unexpected: " + ee);
                }
            }

            try {
                Object arg[] = {
                    invoker
                };
                return (ProxyTarget) proxyConstructor.newInstance(arg);
            }
            catch (InvocationTargetException ee) {
                throw new RuntimeException("unexpected: " + ee);
            }
            catch (InstantiationException ee) {
                throw new RuntimeException("unexpected: " + ee);
            }
            catch (IllegalAccessException ee) {
                throw new RuntimeException("unexpected: " + ee);
            }
        }

        ProxyInvoker newInvoker(final Object target) {
            if (proxyString == null) {
                String s = "Invoker@" + targetTypes[0].getName();
                for (int i = 1; i < targetTypes.length; i++) {
                    s += "," + targetTypes[i].getName();
                }
                proxyString = s;
            }
            return new ProxyInvoker() {

                // (ISSUE: Should this be made subclassable?)
                public Object getTarget() {
                    return target;
                }

                public Class<?>[] getTargetTypes() {
                    return copyTargetTypes();
                }

                public String toString() {
                    return proxyString + "[" + target + "]";
                }

                public Object invoke(Member method, Object values[])
                        throws Invoker.TargetException {
                    return Impl.this.invoke(target, method, values);
                }
            };
        }

        // the heart of a ProxyInvoker:
        Object invoke(Object target, Member method, Object values[])
                throws Invoker.TargetException {

            // Note: We will not invoke the method unless we are
            // expecting it.
            // Thus, we cannot blindly call Method.invoke, but must
            // first
            // check our list of allowed methods.

            try {
                Method methods[] = this.methods; // cache

                // use fast pointer equality (it usually succeeds)
                for (int i = methods.length; --i >= 0;) {
                    if (methods[i] == method) {
                        return methods[i].invoke(target, values);
                    }
                }

                // special case: allow a null method to select the
                // unique one
                if (method == null) {
                    if (methods.length == 1) {
                        return methods[0].invoke(target, values);
                    }
                    throw new IllegalArgumentException(
                        "non-unique method");
                }

                // try the slower form of equality
                for (int i = methods.length; --i >= 0;) {
                    if (methods[i].equals(method)) {
                        return methods[i].invoke(target, values);
                    }
                }

            }
            catch (IllegalAccessException ee) {
                throw new IllegalArgumentException("method access "
                    + method);
            }
            catch (InvocationTargetException ee) {
                Throwable te = ee.getTargetException();
                if (te instanceof Error) {
                    throw (Error) te;
                }
                if (te instanceof RuntimeException) {
                    throw (RuntimeException) te;
                }
                throw new Invoker.TargetException(te);
            }

            throw new IllegalArgumentException("method unexpected "
                + method);
        }

        void makeProxyConstructor() {
            ProxyCompiler pc = new ProxyCompiler(superclass,
                targetTypes, methods);
            try {
                Class type[] = {
                    Invoker.class
                };
                proxyConstructor = pc.getProxyType().getConstructor(
                    type);
            }
            catch (NoSuchMethodException ee) {
                throw new RuntimeException("unexpected: " + ee);
            }
        }
    }
}
