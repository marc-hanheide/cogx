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
package cast.ui.architecture;

import cast.core.CASTException;


/**
 * @author nah
 *
 */
public class CASTUIException extends CASTException {


    /**
	 * 
	 */
	private static final long serialVersionUID = 1523890476882663834L;

	/**
     * 
     */
    public CASTUIException() {
    }

    /**
     * @param _message
     */
    public CASTUIException(String _message) {
        super(_message);
    }

    /**
     * @param _cause
     */
    public CASTUIException(Throwable _cause) {
        super(_cause);
    }

    /**
     * @param _message
     * @param _cause
     */
    public CASTUIException(String _message, Throwable _cause) {
        super(_message, _cause);
    }

}
