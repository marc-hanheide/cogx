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
 */package edu.ksu.cis.bnj.ver3.streams.xml;

import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Document;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import edu.ksu.cis.bnj.ver3.streams.EvidenceStream;


/*!
 * \file Converter_xmlevidence.java
 * \author Jeffrey M. Barber
 */
public class Converter_xmlevidence implements EvidenceStream
{
	EvidenceStream _Writer;
	public void load(InputStream stream, EvidenceStream writer)
	{
		_Writer = writer;

		Document doc;
		DocumentBuilder parser;
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		factory.setValidating(true);
		factory.setNamespaceAware(true);
		//Parse the document
		try
		{
			parser = factory.newDocumentBuilder();
			doc = parser.parse(stream);
		}
		catch (Exception e)
		{
			throw new RuntimeException(e);
		}
		visitDocument(doc);
		System.gc();
	}
	
	public void visitDocument(Node parent)
	{
		NodeList l = parent.getChildNodes();
		if (l == null) throw new RuntimeException("Unexpected end of document!");
		int max = l.getLength();
		for (int i = 0; i < max; i++)
		{
			Node node = l.item(i);
			switch (node.getNodeType())
			{
				case Node.ELEMENT_NODE:
					String name = node.getNodeName();
					if (name.equals("BNJEVIDENCE")) //<evidence>
					{ //$NON-NLS-1$
						_Writer.Start();
						visitDocument(node);
						_Writer.End();
					}
					else if (name.equals("EVIDENCE"))
					{ //$NON-NLS-1$
						_Writer.BeginEvidence();
						visitModel(node);
						_Writer.EndEvidence();
					}
					else
						throw new RuntimeException("Unhandled element " + name);
					break;
				case Node.DOCUMENT_TYPE_NODE:
				case Node.DOCUMENT_NODE:
				case Node.COMMENT_NODE:
				case Node.TEXT_NODE:
					//Ignore this
					break;
				default:
			//if (Settings.DEBUG) System.out.println("Unhandled node " +
			// node.getNodeName());
			}
		}
	}	
	public void visitModel(Node parent)
	{
		
		NodeList l = parent.getChildNodes();
		if (l == null) throw new RuntimeException("Unexpected end of document!");
		int max = l.getLength();
		// Split into two loops so that it can handle forward reference
		for (int i = 0; i < max; i++)
		{
			Node node = l.item(i);
			switch (node.getNodeType())
			{
				case Node.ELEMENT_NODE:
					String name = node.getNodeName();
					if (name.equals("SAMPLE"))
					{

						visitSample(node);
						
						//_Writer.BeginSample(0);
						// visit
						//_Writer.EndSample();
					}
					break;
				case Node.DOCUMENT_TYPE_NODE:
				case Node.DOCUMENT_NODE:
				case Node.COMMENT_NODE:
				case Node.TEXT_NODE:
					//Ignore this
					break;
				default:
			//if (Settings.DEBUG) System.out.println( "Unhandled node " +
			// node.getNodeName());
			}
		}
	}
	
	protected void visitSample(Node parent)
	{
		NodeList l = parent.getChildNodes();
		int max;
		NamedNodeMap attrs = parent.getAttributes();
		int _time = 0;

		if (attrs != null)
		{
			max = attrs.getLength();
			for (int i = 0; i < max; i++)
			{
				Node attr = attrs.item(i);
				String name = attr.getNodeName();
				String value = attr.getNodeValue();
				if (name.equals("TIME"))
				{
					_time = Integer.parseInt(value);
				}
				else
				{
					System.out.println("Unhandled variable property attribute " + name);
				}
			}
		}
		_Writer.BeginSample(_time);
		max = l.getLength();
		for (int i = 0; i < max; i++)
		{
			Node node = l.item(i);
			switch (node.getNodeType())
			{
				case Node.ELEMENT_NODE:
					String name = node.getNodeName();
					if (name.equals("WITNESS"))
					{
						visitWitness(node);
					}
					break;
				case Node.DOCUMENT_NODE:
				case Node.COMMENT_NODE:
				case Node.TEXT_NODE:
					break;
				default:

			}
		}
		_Writer.EndSample();
	}
	protected void visitWitness(Node parent)
	{
		NodeList l = parent.getChildNodes();
		int max;
		String propType = "nature"; //$NON-NLS-1$
		NamedNodeMap attrs = parent.getAttributes();
		String _name = "";
		String _value = "";
		if (attrs != null)
		{
			max = attrs.getLength();
			for (int i = 0; i < max; i++)
			{
				Node attr = attrs.item(i);
				String name = attr.getNodeName();
				String value = attr.getNodeValue();
				if (name.equals("NAME"))
				{
					_name = value;
				}
				else if (name.equals("VALUE"))
				{ 
					_value = value;
				}
					else

				{
					System.out.println("Unhandled variable property attribute " + name);
				}
			}
		}
		_Writer.Witness(_name,_value);
	}
	
	Writer		w				= null;
	public void save(OutputStream os)
	{
		w = new OutputStreamWriter(os);
	}
	
	public void fwrite(String x)
	{
		System.out.println("write: " + x);
		try
		{
			w.write(x);
			w.flush();
		}
		catch (Exception e)
		{
			System.out.println("unable to write?");
		}
	}
	
	public void Start()
	{
		fwrite("<?xml version=\"1.0\" encoding=\"US-ASCII\"?>\n");
		fwrite("<!--\n");
		fwrite("BNJ 3.0 Evidence format\n");
		fwrite("-->\n");
		fwrite("		<!-- DTD for the XMLKVIDENCE-->\n");
		fwrite("<!DOCTYPE BNJEVIDENCE [\n");
		fwrite("<!ELEMENT BNJEVIDENCE ( EVIDENCE )*>\n");
		fwrite("<!ELEMENT EVIDENCE ( SAMPLE )*>\n");
		fwrite("<!ELEMENT SAMPLE ( WITNESS )* >\n");
		fwrite("\t<!ATTLIST SAMPLE TIME CDATA \"0\">\n");
		fwrite("<!ELEMENT WITNESS (#PCDATA)>\n");
		fwrite("\t<!ATTLIST WITNESS NAME CDATA \"Jeff\">\n");
		fwrite("\t<!ATTLIST WITNESS VALUE CDATA \"Rocks\">\n");
		fwrite("]>\n");
		fwrite("<BNJEVIDENCE>\n");		
	}
	
	public void End()
	{
		fwrite("</BNJEVIDENCE>\n");
		try{ w.close(); } catch (Exception e) { }
	}		
	public void BeginEvidence()
	{
		fwrite("\t<EVIDENCE>");
	}
	public void BeginSample(int time)
	{
		fwrite("\t\t<SAMPLE TIME=\"" + time + "\">");
	}
	public void Witness(String name, String value)
	{
		fwrite("\t\t\t<WITNESS NAME=\""+name+"\" VALUE=\""+value+"\"/>");
	}
	public void EndSample()
	{
		fwrite("\t\t</SAMPLE>");
	}
	public void EndEvidence()
	{
		fwrite("\t</EVIDENCE>");
	}
	

}