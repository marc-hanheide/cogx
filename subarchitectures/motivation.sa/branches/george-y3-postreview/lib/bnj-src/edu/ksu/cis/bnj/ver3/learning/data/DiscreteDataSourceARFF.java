/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.bnj.ver3.learning.data;

import java.io.BufferedReader;
import java.io.FileReader;
import edu.ksu.cis.bnj.ver3.core.Discrete;

/*!
 * \file DiscreteDataSourceARFF.java
 * \author Jeffrey M. Barber
 */
public class DiscreteDataSourceARFF implements IDiscreteDataSource
{
	private String 		_Name;
	private String[] 	_ColumnNames;
	private Discrete[]	_ColumnDomains;
	private	int			_NumColumns;
	private short[]		_Data;
	private int			_DataCount;
	
	private int 		_OriginalBytes;
	private int			_NewBytes;
	
	private void buffer(int k)
	{
		String[]   _nextC = new String  [_ColumnNames.length + k];
		Discrete[] _nextD = new Discrete[_ColumnNames.length + k];
		for(int i = 0; i < _ColumnNames.length; i++)
		{
			_nextC[i] = _ColumnNames[i];
			_nextD[i] = _ColumnDomains[i];
		}
		_ColumnNames   = _nextC;
		_ColumnDomains = _nextD;
	}
	
	private void expand()
	{
		System.out.println("Expanding Data!");
		short [] nextData = new short[_Data.length * 2];
		for(int i = 0; i < _Data.length; i++)
			nextData[i] = _Data[i];
		_Data = nextData;
	}
	
	private void finish()
	{
		short [] nextData = new short[_DataCount * _NumColumns];
		System.out.println("removed waste: " + _Data.length + " to " + nextData.length);
		for(int i = 0; i < _DataCount * _NumColumns; i++)
			nextData[i] = _Data[i];
		_Data = nextData;
	}
	
	private void writeData(short[] data)
	{
		if(data.length != _NumColumns)
			System.out.println("BADNESS! <- a data element is bad!");
		int PosWrite = _DataCount * _NumColumns;
		if(PosWrite + _NumColumns >= _Data.length)
			expand();
		for(int i = 0; i < data.length; i++)
			_Data[PosWrite + i] = data[i];
		_DataCount++;
	}
	
	private void addColumn(String name, Discrete D)
	{
		if(_NumColumns >= _ColumnNames.length)
		{
			buffer(100);
		}
		_ColumnNames[_NumColumns] = name;
		_ColumnDomains[_NumColumns] = D;
		_NumColumns++;
	}
	
	private void compact()
	{
		String[]   _nextC = new String  [_NumColumns];
		Discrete[] _nextD = new Discrete[_NumColumns];
		for(int i = 0; i < _NumColumns; i++)
		{
			_nextC[i] = _ColumnNames[i];
			_nextD[i] = _ColumnDomains[i];
		}
		_ColumnNames   = _nextC;
		_ColumnDomains = _nextD;		
	}
	
	private int HeaderLine(String line)
	{
		int lLen = line.length();
		if(lLen >= 9)
		{
			if(line.substring(0,9).toUpperCase().equals("@RELATION"))
			{
				_Name = line.substring(9).trim();
			} else if(line.substring(0,10).toUpperCase().equals("@ATTRIBUTE"))
			{
				String data = line.substring(10).trim();
				// unfold the { , , , } to , , ,
				int k = data.indexOf("{");
				String domainName = data.substring(0,k).trim();
				data = data.substring(k+1);
				k = data.indexOf("}");
				data = data.substring(0,k).trim();

				Discrete D = new Discrete();
				k = data.indexOf(",");
				while(k>0)
				{
					String dName = data.substring(0,k).trim();
					D.addName(dName);
					data = data.substring(k+1).trim();
					k = data.indexOf(",");
				}
				D.addName(data.trim());
				addColumn(domainName, D);
			}
		}
		else if(lLen >= 5)
		{
			if(line.substring(0,5).toUpperCase().equals("@DATA"))
			{
				_Data = new short[_NumColumns * 100];
				compact();
				return 1;
			}
		}
		return 0;
	}
	
	private void DataLine(String line)
	{
		_OriginalBytes += line.length();
		_NewBytes += 2 * _NumColumns;
		
		short[] cell = new short[_NumColumns];
		for(int i = 0; i < _NumColumns; i++)
			cell[i] = -1;
		String data = line + ",";
		int k = data.indexOf(",");
		int idx = 0;
		String val;
		while(k>0)
		{
			val = data.substring(0,k).trim();
			cell[idx] = (short) _ColumnDomains[idx].findName(val);
			data = data.substring(k+1);
			k = data.indexOf(",");
			idx++;
		}
		writeData(cell);
	}
	
	public DiscreteDataSourceARFF(String filename) throws Exception
	{
		_NumColumns = 0;
		_DataCount = 0;
		_ColumnDomains = new Discrete[100];
		_ColumnNames = new String[100];
		
		FileReader F = new FileReader(filename);
		BufferedReader BR = new BufferedReader(F);
		int readMode = 0;
		while(BR.ready())
		{
			if(readMode == 1)
				DataLine(BR.readLine());
			if(readMode == 0)
				readMode = HeaderLine(BR.readLine());
		}
		finish();
		
		System.out.println("Reduction from " + _OriginalBytes + " to " + _NewBytes);
	}
	
	public String getColumnName(int k)
	{
		return _ColumnNames[k];	
	}
	
	public String getValueName(int col, int val)
	{
		return _ColumnDomains[col].getName(val);
	}
	
	public int getNumberColumns()
	{
		return _NumColumns;	
	}

	public int getNumberRows()
	{
		return _DataCount;	
	}
	
	public int findColumn(String name)
	{
		for(int i = 0; i < _NumColumns; i++)
		{
			if(name.trim().toUpperCase().equals(_ColumnNames[i].toUpperCase()))
				return i;
		}
		return 0;
	}
	
	public Discrete getDiscrete(int col)
	{
		return _ColumnDomains[col];
	}
	
	public int[] getQueryRegister()
	{
		int[] q = new int[_NumColumns];
		for(int k = 0; k < _NumColumns; k++)
		{
			q[k] = -1;
		}
		return q;
	}
	
	public void reset(int[] q)
	{
		for(int k = 0; k < _NumColumns; k++)
		{
			q[k] = -1;
		}
	}
	
	public int query(int[] q)
	{
		int Min = 0;
		int Max = _NumColumns-1;
		while(q[Min] == -1 && Min < _NumColumns) Min++;
		while(q[Max] == -1 && Max >= 0) Max--;
		if(Max < Min) return _DataCount;
		
		int cnt = 0;
		if(q.length != _NumColumns) System.out.println("BAD QUERY!!");
		// check cache for compatible query table
		for(int i = 0; i < _DataCount; i++)
		{
			int cellAddress = i * _NumColumns;
			boolean good = true;
			for(int k = Min; k <= Max && good; k++)
			{
				if(q[k] != -1)
				{
					int v = _Data[cellAddress + k];
					good = (v == q[k]);
				}
			}
			if(good) cnt++;
		}
		return cnt;	
	}
}
