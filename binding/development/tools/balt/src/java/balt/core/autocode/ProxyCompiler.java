
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

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

// The following code manages bytecode assembly for dynamic proxy
// generation.

class ProxyCompiler {

    Class superclass;
    Runtime runtime;
    Class targetTypes[];
    Method methods[];

    Class proxyType;

    ProxyCompiler(Class superclass, Class targetTypes[],
            Method methods[]) {
        this.superclass = superclass;
        this.targetTypes = targetTypes;
        this.methods = methods;

        this.runtime = new Runtime();
        this.runtime.targetTypes = targetTypes;
        this.runtime.methods = methods;

        runtime.makeProxyType(this);
    }

    Class getProxyType() {
        return proxyType;
    }

    // this is the only data needed at runtime:
    public static class Runtime extends ClassLoader {

        // These members are common utilities used by ProxyTarget
        // classes.
        // They are all public so they can be linked to from generated
        // code.
        // I.e., they are the runtime support for the code compiled
        // below.
        Class targetTypes[];
        Method methods[];
        ProxyCompiler compiler; // temporary!

        public static final Object NOARGS[] = {};

        public static String toString(Proxies.ProxyTarget target) {
            // This method is not used if one of the target types
            // declare toString.
            Invoker invoker = target.getInvoker();
            return "ProxyTarget[" + invoker + "]";
        }

        public Class[] copyTargetTypes() {
            try {
                return (Class[]) targetTypes.clone();
            }
            catch (IllegalArgumentException ee) {
                return new Class[0];
            }
        }

        public Object invoke(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            try {
                return invoker.invoke(methods[methodNum], values);
            }
            catch (Invoker.TargetException ee) {
                throw ee.getTargetException();
            }
        }

        public boolean invokeBoolean(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Boolean) result).booleanValue();
        }

        public byte invokeByte(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Number) result).byteValue();
        }

        public char invokeChar(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Character) result).charValue();
        }

        public short invokeShort(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Number) result).shortValue();
        }

        public int invokeInt(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Number) result).intValue();
        }

        public long invokeLong(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Number) result).longValue();
        }

        public float invokeFloat(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Number) result).floatValue();
        }

        public double invokeDouble(Invoker invoker, int methodNum,
                Object values[]) throws Throwable {
            Object result = invoke(invoker, methodNum, values);
            return ((Number) result).doubleValue();
        }

        public static Boolean wrap(boolean x) {
            return new Boolean(x);
        }

        public static Byte wrap(byte x) {
            return new Byte(x);
        }

        public static Character wrap(char x) {
            return new Character(x);
        }

        public static Short wrap(short x) {
            return new Short(x);
        }

        public static Integer wrap(int x) {
            return new Integer(x);
        }

        public static Long wrap(long x) {
            return new Long(x);
        }

        public static Float wrap(float x) {
            return new Float(x);
        }

        public static Double wrap(double x) {
            return new Double(x);
        }

        // the class loading part

        void makeProxyType(ProxyCompiler compiler) {
            this.compiler = compiler; // temporary, for use during
            // loading

            byte code[] = compiler.getCode();
            /***********************************************************
             * ---- ---- try { String fname =
             * compiler.getProxyClassName(); fname = fname.substring(1 +
             * fname.lastIndexOf('.')) + ".class"; fname = "/tmp/" +
             * fname; java.io.OutputStream cf = new
             * java.io.FileOutputStream(fname); cf.write(code);
             * cf.close(); System.out.println("wrote "+fname); }
             * catch(java.io.IOException ee) { } //* ---- ----
             */

            compiler.proxyType = defineClass(compiler
                .getProxyClassName(), code, 0, code.length);
            resolveClass(compiler.proxyType);
            // set the Foo$Impl.info pointer to myself
            try {
                Field infoField = compiler.proxyType
                    .getField(INFO_FIELD);
                infoField.set(null, this);
            }
            catch (IllegalAccessException ee) {
                throw new RuntimeException("unexpected: " + ee);
            }
            catch (NoSuchFieldException ee) {
                throw new RuntimeException("unexpected: " + ee);
            }
            compiler = null;
        }

        ClassLoader getTargetClassLoader() {
            for (int i = 0; i < targetTypes.length; i++) {
                ClassLoader cl = targetTypes[i].getClassLoader();
                if (cl != null) {
                    return cl;
                }
            }
            return null;
        }

        public synchronized Class loadClass(String name, boolean resolve)
                throws ClassNotFoundException {
            if (name.endsWith(IMPL_SUFFIX)
                && name.equals(compiler.getProxyClassName())) {
                return compiler.proxyType;
            }
            // delegate to the original class loader
            ClassLoader cl = getTargetClassLoader();
            if (cl == null) {
                return findSystemClass(name);
            }
            return cl.loadClass(name);
        }

        public java.io.InputStream getResourceAsStream(String name) {
            // delegate to the original class loader
            ClassLoader cl = getTargetClassLoader();
            if (cl == null) {
                return getSystemResourceAsStream(name);
            }
            return cl.getResourceAsStream(name);
        }

        public java.net.URL getResource(String name) {
            // delegate to the original class loader
            ClassLoader cl = getTargetClassLoader();
            if (cl == null) {
                return getSystemResource(name);
            }
            return cl.getResource(name);
        }

    }

    // the code generation part

    private static String IMPL_SUFFIX = "$Proxy";

    String getProxyClassName() {
        // Note: We could reasonably put the $Impl class in either
        // of two packges: The package of Proxies, or the same package
        // as the target type. We choose to put it in same package as
        // the target type, to avoid name encoding issues.
        //
        // Note that all infrastructure must be public, because the
        // $Impl class is inside a different class loader.
        String tName = targetTypes[0].getName();
        /*
         * String dName = Dispatch.class.getName(); String pkg =
         * dName.substring(0, 1 + dName.lastIndexOf('.')); return pkg +
         * tName.substring(1 + tName.lastIndexOf('.')) + IMPL_SUFFIX;
         */
        return tName + IMPL_SUFFIX;
    }

    private static String INFO_FIELD = "$info";
    private static String INVOKER_FIELD = "$invoker";

    private static final Class invokeParams[] = {
        Invoker.class, Integer.TYPE, Object[].class
    };
    private static final Class toStringParams[] = {
        Proxies.ProxyTarget.class
    };

    /** Create the implementation class for the given target. */
    private byte[] getCode() {
        String pClass = getProxyClassName();

        int icount = 1; // don't forget ProxyTarget
        for (int i = 0; i < targetTypes.length; i++) {
            Class targetType = targetTypes[i];
            if (targetType.isInterface()) {
                icount++;
            }
        }
        Class interfaces[] = new Class[icount];
        interfaces[0] = Proxies.ProxyTarget.class;
        icount = 1;
        for (int i = 0; i < targetTypes.length; i++) {
            Class targetType = targetTypes[i];
            if (targetType.isInterface()) {
                interfaces[icount++] = targetType;
            }
            else if (!superclass.isAssignableFrom(targetType)) {
                throw new RuntimeException("unexpected: " + targetType);
            }
        }
        ProxyAssembler asm = new ProxyAssembler(pClass, Modifier.PUBLIC
            | Modifier.FINAL, superclass, interfaces);

        Class rClass = ProxyCompiler.Runtime.class;
        asm.addMember(Modifier.PUBLIC + Modifier.STATIC, rClass,
            INFO_FIELD);

        // ProxyTarget implementation
        Class iClass = Invoker.class;
        asm.addMember(Modifier.PRIVATE, iClass, INVOKER_FIELD);
        asm.addMember(Modifier.PUBLIC, iClass, "getInvoker",
            new Class[] {});
        {
            asm.pushLocal(0);
            asm.pushField(asm, INVOKER_FIELD);
            asm.ret();
        }
        asm.addMember(Modifier.PUBLIC, Class[].class, "getTargetTypes",
            new Class[] {});
        {
            asm.pushLocal(0);
            asm.pushField(asm, INFO_FIELD);
            asm.invoke(rClass, "copyTargetTypes", new Class[] {});
            asm.ret();
        }

        boolean haveToString = false;

        // Implement the methods of the target types.
        for (int i = 0; i < methods.length; i++) {
            Method m = methods[i];
            String name = m.getName();
            Class rtype = m.getReturnType();
            Class ptypes[] = m.getParameterTypes();
            if (name.equals("toString") && ptypes.length == 0) {
                haveToString = true;
            }
            asm.addMember(Modifier.PUBLIC + Modifier.FINAL, rtype,
                name, ptypes);
            {
                // $info.invokeBoolean(invoker, i, new Object[]{ ... })
                asm.pushField(asm, INFO_FIELD);
                asm.pushLocal(0);
                asm.pushField(asm, INVOKER_FIELD);
                asm.pushConstant(i);
                // push the arguments
                if (ptypes.length == 0) {
                    asm.pushField(rClass, "NOARGS");
                }
                else {
                    asm.pushConstant(ptypes.length);
                    asm.pushNewArray(Object.class);
                    for (int j = 0; j < ptypes.length; j++) {
                        Class t = ptypes[j];
                        asm.dup();
                        asm.pushConstant(j);
                        asm.pushLocal(1 + j);
                        if (t.isPrimitive()) {
                            asm.invoke(rClass, "wrap", new Class[] {
                                t
                            });
                        }
                        asm.setElement(Object.class);
                    }
                }
                // call the invoker
                String invoke = "invoke";
                if (rtype.isPrimitive() && rtype != Void.TYPE) {
                    String tn = rtype.getName();
                    invoke += Character.toUpperCase(tn.charAt(0))
                        + tn.substring(1);
                }
                asm.invoke(rClass, invoke, invokeParams);
                if (!rtype.isPrimitive() && rtype != Object.class) {
                    asm.checkCast(rtype);
                }
                asm.ret();
            }
        }

        if (!haveToString) {
            asm.addMember(Modifier.PUBLIC, String.class, "toString",
                new Class[] {});
            {
                asm.pushLocal(0);
                asm.invoke(rClass, "toString", toStringParams);
                asm.ret();
            }
        }

        // Put in the constructor:
        asm.addMember(Modifier.PUBLIC, Void.TYPE, "<init>",
            new Class[] {
                iClass
            });
        {
            asm.pushLocal(0);
            asm.invoke(superclass, "<init>", new Class[0]);
            asm.pushLocal(0);
            asm.pushLocal(1);
            asm.setField(asm, INVOKER_FIELD);
            asm.ret();
        }

        return asm.getCode();
    }
}
