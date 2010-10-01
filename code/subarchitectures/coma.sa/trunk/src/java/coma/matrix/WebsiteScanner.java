package coma.matrix;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//der DOMParser von Xerces
import org.apache.xerces.parsers.DOMParser;

// Java-Interfaces for die Knotentypen des DOM
import org.w3c.dom.Document;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

// Fehlerbehandlung bei fehlerhaftem Parsing/Zugriff auf Datei
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.SAXNotRecognizedException;
import org.xml.sax.SAXNotSupportedException;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.io.PrintStream;
import java.net.MalformedURLException;
import java.net.URL;


public class WebsiteScanner {
	
	private Integer m_numHits;
	
	private String elementName;
	private String attributeName;
	private String attributeValue;
	private DOMParser parser;
	
	public WebsiteScanner(String elementName, String attributeName, String attributeValue) {
		parser = new DOMParser();
		try {
			parser.setFeature("http://apache.org/xml/features/nonvalidating/load-external-dtd", false);
			// parser.setFeature("http://apache.org/xml/features/validation/unparsed-entity-checking", false);
		} catch (SAXNotRecognizedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (SAXNotSupportedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		this.elementName = elementName;
		this.attributeName = attributeName;
		this.attributeValue = attributeValue;
		
		this.m_numHits = null;
	}
	
	public Integer scanForHits(String websiteAddress) {
		try {
			if (websiteAddress.startsWith("http")) parser.parse(websiteAddress);
			else parser.parse(new InputSource(new FileInputStream(websiteAddress)));
			Document document = parser.getDocument();
			// System.out.println("traverseAndGetNumHits("+document+","+ elementName+","+ attributeName+ "," + attributeValue+")");
			traverseAndGetNumHits(document, elementName, attributeName, attributeValue);
			if (this.m_numHits==null) this.m_numHits = 0;
		} catch (SAXException e) {
			System.err.println (e);
			if (!(e.toString().contains("&"))) return new Integer (-1);
			try {
				BufferedReader reader;
				if (websiteAddress.startsWith("http")) reader = new BufferedReader(new InputStreamReader(new URL(websiteAddress).openStream()));
				else reader = new BufferedReader(new InputStreamReader(new FileInputStream(websiteAddress)));

				String line = reader.readLine();
				DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-HH:mm:ss:SSS");
				Date date = new Date();
				String currDateID = dateFormat.format(date);

				String cacheFileName = "caches/websiteScanner"+currDateID+".cache";
				FileOutputStream fstream = new FileOutputStream(cacheFileName);
				PrintStream outstream = new PrintStream(fstream);
				while (line != null) {
					outstream.println(line.replaceAll("&", "").replaceAll("/<script", ""));
					//replaceAll("&&", "&amp;&amp;").replaceAll(" & ", " &amp; ").
					//replaceAll("&(#[1-9][0-9]{1,3}|[0-9A-Za-z]+)[^;]+[:blank:]"," &amp; "));
					//replaceAll("&[^;]+[:blank:]", ""));
					line = reader.readLine(); 
				}
				System.out.println("TRYING TO PARSE LOCAL FILE!");
				this.m_numHits = null;
				return scanForHits(cacheFileName);
			} catch (MalformedURLException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Integer returnNumber = (this.m_numHits != null ? this.m_numHits : new Integer(-1));
		this.m_numHits = null;
	    return returnNumber;
	}
	
	
	private void traverseAndGetNumHits(Node node, String elementName, String attributeName, String attributeValue) {
		int type = node.getNodeType();
		if (type == Node.ELEMENT_NODE) {
			if (node.getNodeName().contains(elementName)) {
				NamedNodeMap nnm = node. getAttributes();
				Node namedNode = nnm.getNamedItem(attributeName);
				if (namedNode!=null) {
					if (namedNode.toString().contains(attributeValue)) {
						// System.out.println("<" + node.getNodeName()+ " " + namedNode.toString() + ">" + node.getTextContent());
						String regex = "\\d+[,.]*\\d*[,.]*\\d*"; // one or more numbers
						Pattern p = Pattern.compile(regex);
						Matcher m = p.matcher(node.getTextContent());
						if (m.find()) {
							Integer numHits = Integer.parseInt(m.group().replace(",","").replace(".", ""));
							// System.out.println("HITS: " + numHits);
							m_numHits = numHits;
						}
						else m_numHits = 0;
						return;
					}
				}
			}
		}

		// Verarbeitet die Liste der Kindknoten durch rekursiven Abstieg
		NodeList children = node.getChildNodes();
		for (int i=0; i< children.getLength(); i++) {
			traverseAndGetNumHits(children.item(i), elementName, attributeName, attributeValue);
		}
		if (m_numHits!=null) return;
	}

	
	public static void main (String[] args) throws Exception{
		String websiteAddress = args[0];
		WebsiteScanner myScanner = new  WebsiteScanner("span", "class", "sb_count");
		System.out.println(myScanner.scanForHits(websiteAddress));
	}

	
}
