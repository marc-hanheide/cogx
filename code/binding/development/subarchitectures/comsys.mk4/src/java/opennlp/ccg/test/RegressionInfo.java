///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-4 Jason Baldridge and University of Edinburgh (Michael White)
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////
package opennlp.ccg.test;

import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.synsem.LF;
import org.jdom.*;
import org.jdom.input.*;
import java.io.*;
import java.util.*;

/**
 * Manages the info in a regression test file.
 *
 * @author  Jason Baldridge
 * @author  Michael White
 * @version $Revision: 1.8 $, $Date: 2008/08/07 15:37:55 $
 */
public class RegressionInfo {

    // the grammar
    private Grammar grammar;
    
    // the test items
    private TestItem[] testItems;
    
    /** Test item. */
    public class TestItem {
        /** The test sentence/phrase. */
        public String sentence;
        /** The desired number of parses. */
        public int numOfParses = 1;
        /** Whether the sentence/phrase is known to fail to parse. */
        public boolean knownFailure = false;
        /** The full words for the sentence/phrase, or null if none, formatted by the configured tokenizer. */
        public String fullWords = null;
        /** The LF, in XML, for the sentence/phrase, or null if none. */
        public Element lfElt = null;
        /** Any additionally id info, or null if none. */
        public String info = null;
    }

    /** Reads in the given regression test file. */    
    public RegressionInfo(Grammar grammar, File regressionFile) throws FileNotFoundException {
    	this(grammar, new FileInputStream(regressionFile));
    }
    
    /** Reads in a regression test file from the given input stream. */    
    public RegressionInfo(Grammar grammar, InputStream istr) {
        this.grammar = grammar;
        SAXBuilder builder = new SAXBuilder();
        try {
            Document doc = builder.build(istr);
            Element root = doc.getRootElement();
            List items = root.getChildren("item");
            testItems = new TestItem[items.size()];
            for (int i = 0; i < items.size(); i++) {
                Element item = (Element) items.get(i);
                TestItem testItem = new TestItem();
                testItems[i] = testItem;
                testItem.sentence = item.getAttributeValue("string");
                testItem.numOfParses = Integer.parseInt(item.getAttributeValue("numOfParses"));
                testItem.knownFailure = ("true".equals(item.getAttributeValue("known"))) ? true : false;
                Element fullWordsElt = item.getChild("full-words");
                if (fullWordsElt != null) testItem.fullWords = fullWordsElt.getTextNormalize();
                testItem.lfElt = item.getChild("lf");
                testItem.info = item.getAttributeValue("info");
            }
        } catch (Exception e) {
            throw (RuntimeException) new RuntimeException().initCause(e);
        }
    }

    /** Returns the number of test items. */
    public int numberOfItems() {
        return testItems.length;
    }
    
    /** Returns the test item with the given index. */ 
    public TestItem getItem(int i) {
        return testItems[i];
    }
    
    
    /**
     * Makes an XML test item from the given test item object.
     */
    public static Element makeTestItem(TestItem testItem) {
        Element item = new Element("item");
        item.setAttribute("numOfParses", "" + testItem.numOfParses);
        if (testItem.knownFailure) item.setAttribute("known", "true");
        item.setAttribute("string", testItem.sentence);
        if (testItem.fullWords != null) {
            Element fullWordsElt = new Element("full-words");
            item.addContent(fullWordsElt);
            fullWordsElt.addContent(testItem.fullWords);
        }
        if (testItem.lfElt != null) {
            testItem.lfElt.detach();
            item.addContent(testItem.lfElt);
        }
        if (testItem.info != null) item.setAttribute("info", testItem.info);
        return item;
    }
    
    /**
     * Makes an XML test item with the given string, number of parses and LF, 
     * applying the configured to-XML transformations.
     */
    public Element makeTestItem(String target, int numParses, LF lf) throws IOException { 
        return makeTestItem(grammar, target, numParses, lf);
    }
    
    /**
     * Makes an XML test item with the given string, number of parses and LF, 
     * applying the configured to-XML transformations.
     */
    public static Element makeTestItem(Grammar grammar, String target, int numParses, LF lf) throws IOException { 
        return makeTestItem(grammar, target, numParses, lf, null);
    }

    /**
     * Makes an XML test item with the given string, number of parses, LF and info attribute,  
     * applying the configured to-XML transformations.
     */
    public static Element makeTestItem(Grammar grammar, String target, int numParses, LF lf, String info) throws IOException { 
        Element item = new Element("item");
        item.setAttribute("numOfParses", "" + numParses);
        item.setAttribute("string", target);
        item.addContent(grammar.makeLfElt(lf));
        if (info != null) item.setAttribute("info", info);
        return item;
    }

    /**
     * Adds the given string, number of parses and LF as a test item to the testbed 
     * with the given filename, applying the configured to-XML
     * transformations.
     */
    public static void addToTestbed(Grammar grammar, String target, int numParses, LF lf, String filename) throws IOException { 

        // make test item
        Element item = makeTestItem(grammar, target, numParses, lf);
        
        // ensure dirs exist for filename
        File file = new File(filename);
        File parent = file.getParentFile();
        if (parent != null && !parent.exists()) { parent.mkdirs(); }
        
        // load or make doc
        Document doc; 
        Element root;
        boolean newDoc = false;
        if (file.exists()) {
            // read XML
            SAXBuilder builder = new SAXBuilder();
            try {
                doc = builder.build(file);
            } catch (JDOMException jde) {
                throw (IOException) new IOException().initCause(jde);
            }
            root = doc.getRootElement();
        }
        else {
            doc = new Document();
            root = new Element("regression");
            doc.setRootElement(root);
            newDoc = true;
        }
        
        // append new item
        if (!newDoc) root.addContent("  "); // nb: for some reason, this gets the indenting right
        root.addContent(item);
        
        // save
        FileOutputStream out = new FileOutputStream(file); 
        grammar.serializeXml(doc, out);
        out.close();
    }
}
