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

import balt.corba.data.RemoteDataTranslator;

/**
 * Manages the datatypes that the CAAT framework can use. Only datatypes
 * added to this class are able to be transmitted across machines and
 * languages unless they are primitive types or arrays of primitive
 * types.
 * 
 * @author nah
 */
public class CASTDatatypeManager {

    /**
     * Add an object class to CAAT. This then uses the underlying
     * framework to automatically generate the translation code.
     * 
     * @see framework.corba.data.RemoteDataTranslator#addObjectTranslator(Class);
     * @param _dataClass
     *            The class object of the data. E.g. MyStruct.class.
     */
    public static void addObjectDatatype(Class<?> _dataClass) {
//        System.out.println("Adding " + _dataClass
//            + " datatype to CAST.");

        // registers datatype with translator for cross
        // language/machine support
        RemoteDataTranslator.addObjectTranslator(_dataClass);
    }

    /**
     * Add an array class to CAAT. This then uses the underlying
     * framework to automatically generate the translation code.
     * 
     * @see framework.corba.data.RemoteDataTranslator#addSequenceTranslator(Class,
     *      Class)
     * @param _dataClass
     *            The class object of the data. E.g. MyStruct[].class.
     * @param _helperClass
     *            The helper class for the data. E.g.
     *            MyStructListHelper.class.
     */
    public static void addSequenceDatatype(Class<?> _dataClass,
            Class<?> _helperClass) {
//        System.out.println("Adding " + _dataClass
//            + " datatype to CAAT.");

        // registers datatype with translator for cross
        // language/machine support
        RemoteDataTranslator.addSequenceTranslator(_dataClass,
            _helperClass);
    }

}
