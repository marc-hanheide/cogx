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
import org.omg.CORBA.TCKind;

/**
 * @author nah
 * @param <T>
 */
public interface FrameworkDataTranslator<T> {

    public abstract Any translate(T _data)
            throws FrameworkDataTranslatorException;

    @SuppressWarnings("unchecked")
    public abstract T translate(Any _data)
            throws FrameworkDataTranslatorException;

    public abstract TCKind getTCKind();

    public abstract Class<T> getTransClass();
}