/*
 * CAST - The CoSy Architecture Schema Toolkit
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
package cast.core;

import java.util.*;

/**
 * @author nah
 */
public class SinglePlaceQueue<T> implements Queue<T> {

    private T m_data = null;

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Queue#element()
     */
    public T element() {
        if (m_data == null) {
            throw new NoSuchElementException("list empty");
        }
        return m_data;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Queue#offer(java.lang.Object)
     */
    public boolean offer(T _o) {
        m_data = _o;
        return true;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Queue#peek()
     */
    public T peek() {
        return m_data;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Queue#poll()
     */
    public T poll() {
        T ret = m_data;
        m_data = null;
        return ret;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Queue#remove()
     */
    public T remove() {

        if (m_data == null) {
            throw new NoSuchElementException("list empty");
        }

        T ret = m_data;
        m_data = null;
        return ret;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#add(java.lang.Object)
     */
    public boolean add(T _o) {
        m_data = _o;
        return true;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#addAll(java.util.Collection)
     */
    public boolean addAll(Collection<? extends T> _c) {
        throw new UnsupportedOperationException("bad idea");
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#clear()
     */
    public void clear() {
        m_data = null;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#contains(java.lang.Object)
     */
    public boolean contains(Object _o) {
        if (m_data == null) {
            return false;
        }
        else {
            return m_data.equals(_o);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#containsAll(java.util.Collection)
     */
    public boolean containsAll(Collection<?> _c) {
        return false;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#isEmpty()
     */
    public boolean isEmpty() {
        return m_data == null;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#iterator()
     */
    public Iterator<T> iterator() {
        return null;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#remove(java.lang.Object)
     */
    public boolean remove(Object _o) {
        throw new UnsupportedOperationException("bad idea");
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#removeAll(java.util.Collection)
     */
    public boolean removeAll(Collection<?> _c) {
        throw new UnsupportedOperationException("bad idea");
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#retainAll(java.util.Collection)
     */
    public boolean retainAll(Collection<?> _c) {
        throw new UnsupportedOperationException("bad idea");
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#size()
     */
    public int size() {
        if (m_data == null) {
            return 0;
        }
        else {
            return 1;
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#toArray()
     */
    public Object[] toArray() {
        throw new UnsupportedOperationException("bad idea");
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.Collection#toArray(T[])
     */
    @SuppressWarnings("hiding")
    public <T> T[] toArray(T[] _a) {
        throw new UnsupportedOperationException("bad idea");
    }

}
