//=================================================================
// Copyright (C) 2006-2010 DFKI GmbH / Talking Robots
// Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.dialogue.util;

//=================================================================
// IMPORTS
//=================================================================

import java.util.*;

//=================================================================
// CLASS DOCUMENTATION
//=================================================================

/**
 * Converts an array to an iterator.
 */
public class ArrayIterator <ElementType> implements Iterator {

	//=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	// Array being converted to iterator.
    private final ElementType[] array;

	// Current index into the array.
    private int index = 0;

	// Whether the last element has been removed.
    private boolean lastRemoved = false;

	//=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /**
     * Create an Iterator from an Array.
     *
     * @param array of objects on which to enumerate.
     */
    public ArrayIterator(ElementType[] array){
        this.array = array;
    } // end constructor 

	//=================================================================
    // ITERATOR METHODS
    //=================================================================

    /**
     * Tests if this Iterator contains more elements.
     *
     * @return true if and only if this Iterator object contains at least
     * one more element to provide; false otherwise.
     */
    public boolean hasNext(){
        return (index < array.length);
    } // end hasNext

    /**
     * Returns the next element of this Iterator if this Iterator
     * object has at least one more element to provide.
     *
     * @return the next element of this Iterator.
     * @throws NoSuchElementException if no more elements exist.
     */
    public ElementType next() throws NoSuchElementException {
        if (index >= array.length) throw new NoSuchElementException("Array index: " + index);
        ElementType object = array[index];
        index++;
        lastRemoved = false;
        return object;
    } // end next

    /**
     * Removes the last object from the array by setting the slot in
     * the array to null.
     * This method can be called only once per call to next.
     *
     * @throws IllegalStateException if the next method has not yet been called, or the remove method has already been called after the last call to the next method.
     */
    public void remove(){
        if (index == 0) throw new IllegalStateException();
        if (lastRemoved) throw new IllegalStateException();
        array[index-1] = null;
        lastRemoved = true;
    } // end remove
	
} // end class