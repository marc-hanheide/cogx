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

import org.omg.CORBA.Any;
import org.omg.CORBA.BAD_OPERATION;
import org.omg.CORBA.ORB;
import org.omg.CORBA.TCKind;

/**
 * @author nah
 */
public class IntegerTranslator  implements
FrameworkDataTranslator<Integer>  {

    public Any translate(Integer _data)
            throws FrameworkDataTranslatorException {
        Any a = ORB.init().create_any();
        a.insert_long(_data);
        return a;
    }

    // public Any translate(Integer _data)
    // throws FrameworkDataTranslatorException {
    // Any a = ORB.init().create_any();
    // a.insert_Integer(_data.IntegerValue());
    // return a;
    //    }

    public Integer translate(Any _data)
            throws FrameworkDataTranslatorException {
        try {
            return _data.extract_long();
        }
        catch (BAD_OPERATION e) {
            throw new FrameworkDataTranslatorException(
                "Unable to extract String from Any", e);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see sandbox.FrameworkDataTranslator#getTCKind()
     */
    public TCKind getTCKind() {
        return TCKind.tk_long;
    }

    /* (non-Javadoc)
     * @see balt.corba.data.translation.FrameworkDataTranslator#getTransClass()
     */
    public Class<Integer> getTransClass() {
      return Integer.class;
    }

//    /* (non-Javadoc)
//     * @see balt.corba.data.translation.FrameworkDataTranslator#translate(java.lang.Object)
//     */
//    public Any translate(Integer _data) throws FrameworkDataTranslatorException {
//        try {
//            return new Integer(_data.extract_Integer());
//        }
//        catch (BAD_OPERATION e) {
//            throw new FrameworkDataTranslatorException(
//                "Unable to extract String from Any", e);
//        }
//
//    }

}
