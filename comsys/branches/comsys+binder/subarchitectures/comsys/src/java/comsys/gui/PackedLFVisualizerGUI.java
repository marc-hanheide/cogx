// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
// =================================================================

package comsys.gui;

// =================================================================
// PACKAGE IMPORTS
// =================================================================

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.Hashtable;


// -----------------------------------------------------------------
// JAVA SWING IMPORTS
// -----------------------------------------------------------------
import java.awt.*;
import java.awt.event.*;
import javax.imageio.*;
import java.awt.image.*;
import java.io.File;

import javax.swing.*;
import javax.swing.event.*;

import comsys.datastructs.comsysEssentials.SDRS;
import comsys.datastructs.comsysEssentials.SDRSFormula;
import comsys.utils.SDRSUtils;
import comsys.utils.EventStructureUtils;

import comsys.processing.parse.ActiveIncrCCGParser;

import comsys.components.*;
import comsys.components.gui.*;

import cast.core.CASTData;

// =================================================================
// CLASS DOCUMENTATION 
// =================================================================




public class PackedLFVisualizerGUI 
	extends JFrame
	implements ListSelectionListener, ActionListener
{

	// =================================================================
	// GLOBAL DATA STRUCTURES
	// =================================================================

	// Java SWING variables
	private JLabel label;		// test message at bottom of GUI
	private JLabel picture;		// contains the current graph
	private DefaultListModel listModel; // contains the content for the list of descriptions
	private JList list;			// contains the list of descriptions
	
	// GUI constructs
	private JScrollPane listScrollPane; 
	private JScrollPane pictureScrollPane;
	private JSplitPane  listGraphPane; 
	private JSplitPane  mainSplitPane; 
	private JToolBar	toolbar; 
	private JToolBar	toolbar2; 
	private JToolBar	toolbar3; 

	DotGraphGenerationMonitor monitor;
	
	// the name of the directory in which the files are located 
	String graphsDir = "./graphs/";

	String graphsDirBase = "./graphs/";

	// the name of the directory in which the files for the SDRS are located 
	String graphsDirSDRS = "./graphs/";
	
	Hashtable<String,String> fileDescriptions = new Hashtable<String,String>();
	
	Hashtable<Integer,String> listIndexToFilenames = new Hashtable<Integer,String>();
	
	Hashtable<String,Integer> filenameSeries = new Hashtable<String,Integer>();

	int currentSeriesIndex = 0;
	int maxSeriesIndex = 0;
	String currentFileName = "";
	
	double scaleRatio = 1.0;
	
	int currentFormulaIndex = 1;
	String currentSDRSFileName="sdrs";
	
	
	String lastPictureFileName = "";
	
	// used to access the SDRS data structure
	PackedLFVisualizer plfviz;
	
	boolean logging = true ;

	// =================================================================
	// CONSTUCTOR
	// =================================================================	

	/** Constructs the GUI itself */

	public PackedLFVisualizerGUI (PackedLFVisualizer plfviz) {
		
		this.plfviz = plfviz;
		
		log("Initialize Packed LF Visualizer GUI...");

		JPanel supertoolbar = new JPanel();
		supertoolbar.setLayout(new BorderLayout());
		toolbar = new JToolBar();
		toolbar.setFloatable(false);
		toolbar2 = new JToolBar();
		toolbar2.setFloatable(false);
		toolbar3 = new JToolBar();
		toolbar3.setFloatable(false);
		addButtons();
		supertoolbar.add(toolbar,BorderLayout.WEST);
		supertoolbar.add(toolbar2,BorderLayout.CENTER);
		supertoolbar.add(toolbar3,BorderLayout.EAST);
		
		listModel = new DefaultListModel();
		

		list = new JList(listModel);
		list.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		list.setSelectedIndex(0);
		list.addListSelectionListener(this);


		listScrollPane = new JScrollPane(list);
		
		picture = new JLabel();
		pictureScrollPane = new JScrollPane(picture);
		listGraphPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,listScrollPane,pictureScrollPane);
		listGraphPane.setOneTouchExpandable(true);
		listGraphPane.setDividerLocation(150);
		Dimension minimumSize = new Dimension (200,250);
		listScrollPane.setMinimumSize(minimumSize);
		pictureScrollPane.setMinimumSize(minimumSize);
		listGraphPane.setPreferredSize(new Dimension(500,400));
		listGraphPane.setBorder(null);

		label = new JLabel("Select graph(s) for description; use toolbar to navigate through series",JLabel.CENTER);
		mainSplitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT,listGraphPane,label);
		mainSplitPane.setOneTouchExpandable(true);
		mainSplitPane.setDividerLocation(360);
		mainSplitPane.setPreferredSize(new Dimension(700,400));
		mainSplitPane.setResizeWeight(1.0);
		
		JFrame frame = new JFrame("PackedLF & SDRS Visualization");
		frame.addWindowListener(new WindowAdapter() { public void windowClosing(WindowEvent e) {System.exit(0);}});
		frame.getContentPane().add(supertoolbar,BorderLayout.NORTH);			
		frame.getContentPane().add(mainSplitPane,BorderLayout.CENTER);
		frame.pack();
		frame.setVisible(true);
		
		if (plfviz.generatePNG) {
		monitor = new DotGraphGenerationMonitor(this);
		monitor.start();
		}

		log("Initialization successful");
	} // end constructor

	/** Adds the buttons to the toolbar */

	public void addButtons () { 
		JButton button = null;
		
		button = new JButton ("-1 frame");
		button.setToolTipText("Move one down in a series");
		button.addActionListener(this);
		button.setActionCommand("decr");
		toolbar.add(button);
		
		button = new JButton ("+1 frame");
		button.setToolTipText("Move one up in a series");
		button.addActionListener(this);
		button.setActionCommand("incr");
		toolbar.add(button);
		
		JLabel label = new JLabel("          ");
		toolbar.add(label);
		
		JLabel label2 = new JLabel("Discourse structure:   ");
		toolbar2.add(label2);
		
		button = new JButton (" First ");
		button.setToolTipText("Go to the first SDRS formula in the discourse structure");
		button.addActionListener(this);
		button.setActionCommand("firstDisc");
		toolbar2.add(button);

		JLabel label3 = new JLabel(" ");
		toolbar2.add(label3);
		
		button = new JButton ("< Previous ");
		button.setToolTipText("Go to the previous SDRS formula in the discourse structure");
		button.addActionListener(this);
		button.setActionCommand("previousDisc");
		toolbar2.add(button);

		JLabel label4 = new JLabel(" ");
		toolbar2.add(label4);
		
		button = new JButton (" Next >");
		button.setToolTipText("Go to the next SDRS formula in the discourse structure");
		button.addActionListener(this);
		button.setActionCommand("nextDisc");
		toolbar2.add(button);
		
		JLabel label5 = new JLabel(" ");
		toolbar2.add(label5);
		
		button = new JButton (" Last ");
		button.setToolTipText("Go to the last SDRS formula in the discourse structure");
		button.addActionListener(this);
		button.setActionCommand("lastDisc");
		toolbar2.add(button);
		
		JLabel label6 = new JLabel("      ");
		toolbar2.add(label6);

		button = new JButton (" Full ");
		button.setToolTipText("Generate the full discourse structure and spatio-temporal representation and write the file in " + graphsDir);
		button.addActionListener(this);
		button.setActionCommand("fullDisc");
		toolbar2.add(button);
		
		button = new JButton (" - ");
		button.setToolTipText("Zoom out");
		button.addActionListener(this);
		button.setActionCommand("zoomIn");
		toolbar3.add(button);
		
		button = new JButton (" + ");
		button.setToolTipText("Zoom in");
		button.addActionListener(this);
		button.setActionCommand("zoomOut");
		toolbar3.add(button);
		
	} // end addButtons
	
	// =================================================================
	// AWT EVENT METHODS
	// =================================================================
	
	/** Listens to events on the list */
	
	public void valueChanged(ListSelectionEvent e) {
		log("Change in list selection");
		if (e.getValueIsAdjusting()) { return; }
		
		JList theList = (JList) e.getSource();
		if (theList.isSelectionEmpty()) { 
			picture.setIcon(null);
		} else { 
			int index=theList.getSelectedIndex();
			String uniqueFileName = listIndexToFilenames.get(new Integer(index));
			if (uniqueFileName != null) { 
				Integer seriesNum = filenameSeries.get(uniqueFileName);
				String pictureFileName = uniqueFileName; 
				currentFileName = pictureFileName;
				if (seriesNum != null) { 
					label.setText("Filename: "+uniqueFileName+" (series 0/"+seriesNum+")");
					pictureFileName = pictureFileName+"-0.png";
					maxSeriesIndex = seriesNum.intValue();
					
				} else { 
					label.setText("Filename: "+uniqueFileName);		
					pictureFileName=pictureFileName+".png";		
				} // end if..else check for series
				ImageIcon newImage = new ImageIcon (graphsDir+pictureFileName); 
				picture.setIcon(newImage);
				picture.setPreferredSize(new Dimension(newImage.getIconWidth(), newImage.getIconHeight()));				
				picture.revalidate();
				currentSeriesIndex = 0;
			} 
		} // end if..else check for matching picture 
	} 

	/** Listens to events on the buttons */

	public void actionPerformed (ActionEvent e) { 
		// do something with the buttons
		if (e.getActionCommand().equals("decr")) { 
			if (currentSeriesIndex > 0 && maxSeriesIndex != 0) { 
				log("Decrease button clicked");	
				currentSeriesIndex--;
				String pictureFileName = currentFileName+"-"+currentSeriesIndex+".png";
				label.setText("Filename: "+currentFileName+" (series "+currentSeriesIndex+"/"+maxSeriesIndex+")");	 
				showImage(graphsDir+pictureFileName);
			} // end if..check whether we can decrease
		} else if (e.getActionCommand().equals("incr")) { 
			log("Increase button clicked");			
			if (currentSeriesIndex < maxSeriesIndex) { 
				currentSeriesIndex++;
				String pictureFileName = currentFileName+"-"+currentSeriesIndex+".png";	
				label.setText("Filename: "+currentFileName+" (series "+currentSeriesIndex+"/"+maxSeriesIndex+")");	 								
				showImage(graphsDir+pictureFileName);
			} 
		} 
		else if (e.getActionCommand().equals("firstDisc")) {
			log("firstDic button clicked");			
			int sdrsIncr = plfviz.getSDRSIncr();
			String pictureFileName = currentSDRSFileName+"-"+0+".png";	
			label.setText("SDRS discourse structure ("+1+"/"+sdrsIncr+")");	 								
			showImage(graphsDirSDRS+pictureFileName);
			currentFormulaIndex=0;
		}
		
		else if (e.getActionCommand().equals("lastDisc")) {
			log("lastDisc button clicked");			
			int sdrsIncr = plfviz.getSDRSIncr();
			int sdrsIncr2 = sdrsIncr-1;
			String pictureFileName = currentSDRSFileName+"-"+sdrsIncr2+".png";	
			label.setText("SDRS discourse structure ("+sdrsIncr+"/"+sdrsIncr+")");	 								
			showImage(graphsDirSDRS+pictureFileName);
			currentFormulaIndex = sdrsIncr2;
		}
		else if (e.getActionCommand().equals("previousDisc")) { 
			log("previousDisc button clicked");			
			int sdrsIncr = plfviz.getSDRSIncr();
			if (currentFormulaIndex > 0 && sdrsIncr > 0) { 
				currentFormulaIndex--;
				String pictureFileName = currentSDRSFileName+"-"+currentFormulaIndex+".png";
				log("pictureFileName used: " + pictureFileName);
				label.setText("SDRS discourse structure ("+(new Integer(currentFormulaIndex+1)).toString() +"/"+sdrsIncr+")");	 
				showImage(graphsDirSDRS+pictureFileName);
			}

			} // end if..check whether we can decrease
		else if (e.getActionCommand().equals("nextDisc")) { 
			int sdrsIncr = plfviz.getSDRSIncr();
			log("nextDisc button clicked");			
			if (currentFormulaIndex < sdrsIncr-1) { 
				currentFormulaIndex++;
				String pictureFileName = currentSDRSFileName+"-"+currentFormulaIndex+".png";	
				log("pictureFileName used: " + pictureFileName);
				showImage(graphsDirSDRS+pictureFileName);
			} 
		}
		else if (e.getActionCommand().equals("fullDisc")) {
			log("fullDisc button clicked");	
			String fullDisc = "fullDisc";
			
			String pictureFileName = fullDisc+".png";	
			if (plfviz.lastReceivedSDRS != null && plfviz.lastReceivedSTR != null) {
				EventStructureUtils.fullDiscAndSpatioTemporalRepresentationToGraph
				(plfviz.lastReceivedSDRS, plfviz.lastReceivedSTR, graphsDirBase + fullDisc, plfviz.generatePNG);
			}
			else if (plfviz.lastReceivedSDRS != null) {
				SDRSUtils.SDRSToGraph(plfviz.lastReceivedSDRS, graphsDirBase + fullDisc, plfviz.generatePNG);
			}
			
			label.setText("SDRS full discourse structure and spatio-temporal representation");	 								
		}
		
		else if (e.getActionCommand().equals("zoomIn")) { 
			scaleRatio = scaleRatio*1.3;
			showImage(lastPictureFileName);
		}
		else if (e.getActionCommand().equals("zoomOut")) { 
			scaleRatio = scaleRatio*(1/1.3);
			showImage(lastPictureFileName);
		}
	} // end actionPerformed
	
	
	private void showImage(String pictureFileName) {
		try {
			BufferedImage imageSrc = ImageIO.read(new File(pictureFileName));
			log("image size:" + (int)(imageSrc.getWidth()/scaleRatio));
			Image img = imageSrc.getScaledInstance((int)(imageSrc.getWidth()/scaleRatio), (int)(imageSrc.getHeight()/scaleRatio) ,Image.SCALE_SMOOTH);
			ImageIcon newImage = new ImageIcon (img);  
			picture.setIcon(newImage);
			picture.setPreferredSize(new Dimension(newImage.getIconWidth(), (newImage.getIconHeight())));				
			picture.revalidate();
			lastPictureFileName= pictureFileName;
		}
		catch (Exception exp) {
			log("Warning, file not ready yet!");
		}
	}

	// =================================================================
	// ACCESS METHODS
	// =================================================================

	/**
		The method <i>add</i> adds a filename and a description. 
		A filename in a series is packedlf-CASTDataID:CASTDATATYPE:data-SERIESNUMBER, 
		if not in a series it misses the "-SERIESNUMBER"
	*/ 


	public void add (String fileName, String description) { 
		log("adding a filename");
		int firstDash = fileName.indexOf("-");
		int lastDash  = fileName.lastIndexOf ("-");
		String uniqueFileName = "";
		if (firstDash != lastDash) { 
			// series
			uniqueFileName = fileName.substring(0,lastDash);
			if (filenameSeries.containsKey(uniqueFileName)) { 
				int seqNum = ((Integer)filenameSeries.get(uniqueFileName)).intValue();
				seqNum++;
				filenameSeries.put(uniqueFileName,new Integer(seqNum));				
			} else {
				filenameSeries.put(uniqueFileName,new Integer(0));
			} // end if..else check whether already present
		} else {
			// no series
			uniqueFileName = fileName;
			filenameSeries.put(uniqueFileName,null);
		} // end if..else check for series
		if (!fileDescriptions.containsKey(uniqueFileName)) { 
			fileDescriptions.put(uniqueFileName,description);
			listModel.addElement("\""+description+"\"");
			int size = listModel.getSize();
			listIndexToFilenames.put(new Integer(size-1),uniqueFileName);
			list.setSelectedIndex(size);
		} // end if..check whether already present
	} // end add

	public void setGraphsDir (String dir) { 
		graphsDir = dir;
		if (!(new File(graphsDir)).isDirectory()) {
			(new File(graphsDir)).mkdir();
			log("new directory " + graphsDir + " created");
		}
		else 
			log("directory " + graphsDir + "already exists");
	} // end setGraphsDir


	public void setGraphsDirSDRS (String dir) { 
		graphsDirSDRS = dir;
		if (!(new File(graphsDirSDRS)).isDirectory()) {
			(new File(graphsDirSDRS)).mkdir();
			log("new directory " + graphsDirSDRS + " created");
		}
		else
			log("directory " + graphsDirSDRS + "already exists");
	} // end setGraphsDir


	public void setLabelText(String str) {
		label.setText(str);
	}
	
	public String getLabelText() { return label.getText(); }
	
	private void log(String str) {
		if (logging) {
			System.out.println("[LOG packedLFVisualizer] " +str);
		}
	}
	
}