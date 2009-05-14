
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

import java.lang.reflect.Member;

/**
 * An invoker is a target of method calls, where the calls are expressed
 * not as primitive Java method invocations, but according to the
 * conventions of the Core Reflection API. Invokers are designed to be
 * used along with the Core Reflection API.
 * <p>
 * The Invoker.invoke operation is similar to
 * java.lang.reflect.Method.invoke, except that the object (or objects)
 * which receives the message is hidden behind the invoker. Also, unlike
 * Method.invoke, the action of the Invoker.invoke operation is
 * completely under programmer control, because Invoker.invoke is an
 * interface method, not a native method.
 * <p>
 * You can wrap an invoker around an object so that the invoker passes
 * all method calls down to the object. Such an invoker is called a
 * <em>proxy invoker</em> for that object.
 * <p>
 * You can also wrap a new object around an invoker, so that the object
 * implements some given interface (or interfaces), and passes all
 * method calls up to the invoker. Such an object is called a
 * <em>proxy target object</em> for that invoker.
 * <p>
 * You can do more complex tasks with invokers, such as passing each
 * method call through a network connection before it reaches its target
 * object. You can also filter or replicate method invocations. You can
 * even execute the the invocations interpretively, without ever calling
 * the method on a "real" Java object.
 * <p>
 * 
 * @see java.lang.reflect.Method.invoke
 * @see Invoker.invoke
 * @see Proxies.newInvoker
 * @see Proxies.newTarget
 */
public interface Invoker {

    /**
     * Invoke the given operation on the target with the given
     * arguments.
     * <p>
     * All checked exceptions are wrapped in a single wrapper type, and
     * must be properly unwrapped by the caller.
     * <p>
     * (Note: The first argument is typically a Method, but in general
     * it can be any "cookie" agreed upon by the caller and callee that
     * implements the Member interface.)
     * 
     * @see java.lang.reflect.Method.invoke
     */
    Object invoke(Member method, Object values[])
            throws Invoker.TargetException;

    // The first argument is typed as the open-ended type Member
    // to allow re-use of invoker facilities with method-like entities
    // such as Java Beans properties and script functions.

    /**
     * Return the types (interfaces) for which this Invoker processes
     * method invocations, or null if there are no specific types.
     * 
     * @see Proxies.newTarget
     */
    Class<?>[] getTargetTypes();

    /**
     * Wrapper class for all invocation exceptions.
     */
    public class TargetException extends Exception {

        /**
         * 
         */
        private static final long serialVersionUID = 2144929502394654564L;
        private Throwable target;

        protected TargetException() {
            super();
        }

        public TargetException(String s) {
            super(s);
        }

        public TargetException(Throwable target) {
            super();
            this.target = target;
        }

        public TargetException(Throwable target, String s) {
            super(s);
            this.target = target;
        }

        /**
         * Get the thrown target exception.
         */
        public Throwable getTargetException() {
            return target;
        }

    }
}
