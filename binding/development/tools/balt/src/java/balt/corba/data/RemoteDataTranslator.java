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
package balt.corba.data;

import java.util.HashMap;

import org.omg.CORBA.Any;
import org.omg.CORBA.BooleanSeqHelper;
import org.omg.CORBA.CharSeqHelper;
import org.omg.CORBA.DoubleSeqHelper;
import org.omg.CORBA.FloatSeqHelper;
import org.omg.CORBA.LongLongSeqHelper;
import org.omg.CORBA.LongSeqHelper;
import org.omg.CORBA.OctetSeqHelper;
import org.omg.CORBA.ShortSeqHelper;
import org.omg.CORBA.StringSeqHelper;

import balt.corba.data.translation.BooleanTranslator;
import balt.corba.data.translation.ByteTranslator;
import balt.corba.data.translation.CharTranslator;
import balt.corba.data.translation.DoubleTranslator;
import balt.corba.data.translation.FloatTranslator;
import balt.corba.data.translation.FrameworkDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.corba.data.translation.GenericObjectTranslator;
import balt.corba.data.translation.GenericSequenceTranslator;
import balt.corba.data.translation.IntTranslator;
import balt.corba.data.translation.IntegerTranslator;
import balt.corba.data.translation.LongTranslator;
import balt.corba.data.translation.ShortTranslator;
import balt.corba.data.translation.StringTranslator;

/**
 * @author nah
 */
public class RemoteDataTranslator {

	// because of Java generics don't support primitive data types,
	// we have to do this the hard way
	private static final BooleanTranslator BOOL_TRANSLATOR = new BooleanTranslator();
	private static final ByteTranslator BYTE_TRANSLATOR = new ByteTranslator();
	private static final CharTranslator CHAR_TRANSLATOR = new CharTranslator();
	private static final DoubleTranslator DOUBLE_TRANSLATOR = new DoubleTranslator();
	private static final FloatTranslator FLOAT_TRANSLATOR = new FloatTranslator();
	private static final IntTranslator INT_TRANSLATOR = new IntTranslator();
	private static final IntegerTranslator INTEGER_TRANSLATOR = new IntegerTranslator();
	private static final LongTranslator LONG_TRANSLATOR = new LongTranslator();

	private static HashMap<Class, FrameworkDataTranslator> m_translatorClassMap;

	private static final ShortTranslator SHORT_TRANSLATOR = new ShortTranslator();

	static {
		m_translatorClassMap = new HashMap<Class, FrameworkDataTranslator>();

		// add receiving objects for built-ins
		addTranslators(new StringTranslator());
		addTranslators(BOOL_TRANSLATOR);
		addTranslators(INTEGER_TRANSLATOR);

		// add build-in sequence translators
		addSequenceTranslator(int[].class, LongSeqHelper.class);
		addSequenceTranslator(short[].class, ShortSeqHelper.class);
		addSequenceTranslator(long[].class, LongLongSeqHelper.class);
		addSequenceTranslator(double[].class, DoubleSeqHelper.class);
		addSequenceTranslator(float[].class, FloatSeqHelper.class);
		addSequenceTranslator(byte[].class, OctetSeqHelper.class);
		addSequenceTranslator(boolean[].class, BooleanSeqHelper.class);
		addSequenceTranslator(char[].class, CharSeqHelper.class);
		addSequenceTranslator(String[].class, StringSeqHelper.class);
	}

	/**
	 * 
	 */
	public RemoteDataTranslator() {

	}

	private static void addTranslatorsToMaps(FrameworkDataTranslator<?> dt) {
		m_translatorClassMap.put(dt.getTransClass(), dt);
	}

	public static <T> void addObjectTranslator(Class<T> _dataClass) {

		if (!m_translatorClassMap.containsKey(_dataClass)) {
			FrameworkDataTranslator<T> dt = new GenericObjectTranslator<T>(
					_dataClass);
			addTranslatorsToMaps(dt);
		}
	}

	/**
	 * @param <T>
	 * @param _dataClass
	 * @param _helperClass
	 */
	public static <T> void addSequenceTranslator(Class<T> _dataClass,
			Class _helperClass) {
		if (!m_translatorClassMap.containsKey(_dataClass)) {
			FrameworkDataTranslator<T> dt = new GenericSequenceTranslator<T>(
					_dataClass, _helperClass);
			addTranslatorsToMaps(dt);
		}
	}

	public static void addTranslators(FrameworkDataTranslator<?> dt) {
		addTranslatorsToMaps(dt);
	}

	public static boolean translateBoolFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return BOOL_TRANSLATOR.translate(_a);
	}

	public static byte translateByteFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return BYTE_TRANSLATOR.translate(_a);
	}

	public static char translateCharFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return CHAR_TRANSLATOR.translate(_a);
	}

	public static double translateDoubleFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return DOUBLE_TRANSLATOR.translate(_a);
	}

	public static float translateFloatFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return FLOAT_TRANSLATOR.translate(_a);
	}

	@SuppressWarnings("unchecked")
	public static <T> T translateFromAny(Any _a, Class<T> _dataClass)
			throws FrameworkDataTranslatorException {

		FrameworkDataTranslator<T> translator = m_translatorClassMap
				.get(_dataClass);

		if (translator == null) {

			addObjectTranslator(_dataClass);
			translator = m_translatorClassMap.get(_dataClass);
			assert (translator != null);
		}

		return translator.translate(_a);

	}

	public static int translateIntFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return INT_TRANSLATOR.translate(_a);
	}

	public static long translateLongFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return LONG_TRANSLATOR.translate(_a);
	}

	public static short translateShortFromAny(Any _a)
			throws FrameworkDataTranslatorException {
		return SHORT_TRANSLATOR.translate(_a);
	}

	public static Any translateToAny(boolean _b)
			throws FrameworkDataTranslatorException {
		return BOOL_TRANSLATOR.translate(_b);
	}

	public static Any translateToAny(Boolean _b)
			throws FrameworkDataTranslatorException {
		return BOOL_TRANSLATOR.translate(_b);
	}

	public static Any translateToAny(byte _b)
			throws FrameworkDataTranslatorException {
		return BYTE_TRANSLATOR.translate(_b);
	}

	public static Any translateToAny(char _b)
			throws FrameworkDataTranslatorException {
		return CHAR_TRANSLATOR.translate(_b);
	}

	public static Any translateToAny(double _b)
			throws FrameworkDataTranslatorException {
		return DOUBLE_TRANSLATOR.translate(_b);
	}

	public static Any translateToAny(float _b)
			throws FrameworkDataTranslatorException {
		return FLOAT_TRANSLATOR.translate(_b);
	}

	public static Any translateToAny(int _i)
			throws FrameworkDataTranslatorException {
		return INT_TRANSLATOR.translate(_i);
	}

	public static Any translateToAny(long _l)
			throws FrameworkDataTranslatorException {
		return LONG_TRANSLATOR.translate(_l);
	}

	public static Any translateToAny(short _s)
			throws FrameworkDataTranslatorException {
		return SHORT_TRANSLATOR.translate(_s);
	}

	public static <T> FrameworkDataTranslator<?> getTranslator(Class<T> _cls) {
		return m_translatorClassMap.get(_cls);
	}

	@SuppressWarnings("unchecked")
	public static <T> Any translateToAny(T _o)
			throws FrameworkDataTranslatorException {

		try {
			Class<?> cls = _o.getClass();

			FrameworkDataTranslator<T> translator = m_translatorClassMap
					.get(cls);

			if (translator == null) {
				addObjectTranslator(cls);
				translator = m_translatorClassMap.get(cls);
				assert (translator != null);
			}

			return translator.translate(_o);
		}
		catch (Throwable e) {
			System.err.println("\n\n");
			System.err.println("Unable to translate from type: " + _o.getClass());
			System.err.println("\n\n");
			e.printStackTrace();
			System.err.println("\n\n");
			System.err.println("Unable to translate from type: " + _o.getClass());
			System.err.println("Check java object for nulls");
			System.err.println("\n\n");
			System.exit(0);
			//keep compiler quiet
			return null;
		}
	}

}
