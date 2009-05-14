package org.cognitivesystems.comsys.data;

import java.io.Serializable;

import org.omg.CORBA.Any;
import org.omg.CORBA.BAD_OPERATION;
import org.omg.CORBA.BAD_PARAM;
import org.omg.CORBA.DATA_CONVERSION;
import org.omg.CORBA.MARSHAL;
import org.omg.CORBA.Object;
import org.omg.CORBA.TypeCode;
import org.omg.CORBA.portable.InputStream;
import org.omg.CORBA.portable.OutputStream;

public class CacheElement extends Any {

	String CacheElement = "";
	
	public CacheElement (String CacheElement) {
		this.CacheElement = CacheElement;
	}
	
	@Override
	public InputStream create_input_stream() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public OutputStream create_output_stream() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean equal(Any arg0) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Object extract_Object() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public TypeCode extract_TypeCode() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Serializable extract_Value() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Any extract_any() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean extract_boolean() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public char extract_char() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double extract_double() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public float extract_float() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int extract_long() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public long extract_longlong() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public byte extract_octet() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public short extract_short() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public String extract_string() throws BAD_OPERATION {
		return CacheElement;
	}

	@Override
	public int extract_ulong() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public long extract_ulonglong() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public short extract_ushort() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public char extract_wchar() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public String extract_wstring() throws BAD_OPERATION {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void insert_Object(Object arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_Object(Object arg0, TypeCode arg1) throws BAD_PARAM {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_TypeCode(TypeCode arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_Value(Serializable arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_Value(Serializable arg0, TypeCode arg1) throws MARSHAL {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_any(Any arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_boolean(boolean arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_char(char arg0) throws DATA_CONVERSION {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_double(double arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_float(float arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_long(int arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_longlong(long arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_octet(byte arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_short(short arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_string(String arg0) throws DATA_CONVERSION, MARSHAL {
		CacheElement = arg0;
	}

	@Override
	public void insert_ulong(int arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_ulonglong(long arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_ushort(short arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_wchar(char arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void insert_wstring(String arg0) throws MARSHAL {
		// TODO Auto-generated method stub

	}

	@Override
	public void read_value(InputStream arg0, TypeCode arg1) throws MARSHAL {
		// TODO Auto-generated method stub

	}

	@Override
	public TypeCode type() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void type(TypeCode arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void write_value(OutputStream arg0) {
		// TODO Auto-generated method stub

	}
	
	

}
