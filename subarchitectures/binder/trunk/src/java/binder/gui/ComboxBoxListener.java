// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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

package binder.gui;

import java.awt.BorderLayout;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Vector;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollBar;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.border.Border;
import javax.swing.border.TitledBorder;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

public class ComboxBoxListener implements ActionListener {
	
	InsertProxyWindow insertProxyWindow;
	
	int nbFeatureBoxesShown = 0;
	int nbFeatureValuesBoxesShown = 0;
	
	Vector<JPanel> curFeaturePanels;
	HashMap<Integer,Vector<JPanel>> curFeatureValuePanels;
	
	
	public boolean LOGGING = true;
	private static Logger logger = ComponentLogger.getLogger(ComboxBoxListener.class);

	
	public ComboxBoxListener(InsertProxyWindow insertProxyWindow) {
		this.insertProxyWindow = insertProxyWindow;
		curFeaturePanels = new Vector<JPanel>();
		curFeatureValuePanels = new HashMap<Integer,Vector<JPanel>>();
		
	}
	
	public void actionPerformed(ActionEvent event) {
		log("OK ACTION RECEIVED");
        JComboBox cb = (JComboBox)event.getSource();
       	if (cb.getName().equals("nbFeatures")) {
        String nbFeats = (String)cb.getSelectedItem();
        nbFeatureBoxesShown = Integer.parseInt(nbFeats);
        log("Number of features to show: " + nbFeatureBoxesShown);
        
        for (Enumeration<JPanel> e = curFeaturePanels.elements(); e.hasMoreElements(); ) {
        	JPanel prevFeatPanel = e.nextElement();
        	insertProxyWindow.proxyPanel.remove(prevFeatPanel);
        	
        }
        curFeaturePanels.removeAllElements();
        
    	for (int i = 0 ; i < nbFeatureBoxesShown ; i ++) {
    		
    		JPanel freespacePanel = new JPanel();
    		JLabel freespace = new JLabel(" ");
    		freespacePanel.add(freespace);
    		insertProxyWindow.proxyPanel.add(freespacePanel);
      		curFeaturePanels.add(freespacePanel); 
      		
      		JPanel featurePanel = new JPanel();
      		featurePanel.setLayout(new BoxLayout(featurePanel, BoxLayout.Y_AXIS));
      		
      		JPanel featureLabelPanel = new JPanel(new BorderLayout());
    		JLabel featureLabelLabel = new JLabel("Feature label: ");
    		featureLabelLabel.setFont(new Font("Dialog", Font.PLAIN, 12));
    		featureLabelPanel.add(featureLabelLabel, BorderLayout.WEST);
    		JTextField featureLabel = new JTextField(8);
    		featureLabel.setText("featLabel"+(i+1));
    		featureLabel.setName("featLabel");
    		featureLabel.setFont(new Font("Dialog", Font.PLAIN, 12));
    		featureLabelLabel.setLabelFor(featureLabel);
    		featureLabelPanel.add(featureLabel, BorderLayout.EAST);
    		featurePanel.add(featureLabelPanel);
 
  			freespace = new JLabel(" ");
   			freespace.setFont(new Font("Dialog", Font.PLAIN, 8));
   			featurePanel.add(freespace);
   			
    		JPanel nbFeatureValuesPanel = new JPanel(new BorderLayout());
    		JLabel nbFeatureValuesLabel = new JLabel("Number of alternative feature values: ");
    		nbFeatureValuesLabel.setFont(new Font("Dialog", Font.PLAIN, 12));

    		nbFeatureValuesPanel.add(nbFeatureValuesLabel, BorderLayout.WEST);
    		String[] nbFeaturesValues = {"0", "1", "2", "3", "4", "5"};
    		JComboBox nbFeaturesValuesBox = new JComboBox(nbFeaturesValues);
    		nbFeaturesValuesBox.setFont(new Font("Dialog", Font.PLAIN, 12));
    		nbFeaturesValuesBox.setName("nbFeatureValues"+i);
    		nbFeatureValuesLabel.setLabelFor(nbFeaturesValuesBox);
    		nbFeaturesValuesBox.addActionListener(this);
    		nbFeatureValuesPanel.add(nbFeaturesValuesBox, BorderLayout.EAST);
    		featurePanel.add(nbFeatureValuesPanel);
    		
   			TitledBorder border = BorderFactory.createTitledBorder("Feature " + (i+1));
   			border.setTitleFont(new Font("Dialog", Font.BOLD, 12));
   			featurePanel.setBorder(BorderFactory.createCompoundBorder(border, BorderFactory.createEmptyBorder(5,5,5,5)));
   			featurePanel.setName("feature"+i);
      		
   			insertProxyWindow.proxyPanel.add(featurePanel);
      		curFeaturePanels.add(featurePanel);
      	}


       	}
       	else if (cb.getName().contains("nbFeatureValues")) {
       		String nbFeats = (String)cb.getSelectedItem();
       		nbFeatureValuesBoxesShown = Integer.parseInt(nbFeats);
       		log("Number of features values to show: " + nbFeatureValuesBoxesShown);

       		String featureNumberStr = (String) cb.getName().replace("nbFeatureValues", "");
       		int featureNumber = (Integer.parseInt(featureNumberStr) *2 ) + 1;
       		JPanel curFeaturePanel = curFeaturePanels.get(featureNumber);
       		log("current feature to examine: " + featureNumber);
       	
       		Vector<JPanel> featValuePanels = curFeatureValuePanels.get(featureNumber);

       		if (featValuePanels != null) {
       			log("OK, non null");
       			log("size: " + featValuePanels.size());
       			for (Enumeration<JPanel> e = featValuePanels.elements(); e.hasMoreElements(); ) {
       				JPanel prevFeatValuePanel = e.nextElement();
       				curFeaturePanel.remove(prevFeatValuePanel);
       			}
       			featValuePanels.removeAllElements();
       		}
       		else {
       			featValuePanels = new Vector<JPanel>();
       			curFeatureValuePanels.put(featureNumber, featValuePanels);
       		}

       		for (int i = 0 ; i < nbFeatureValuesBoxesShown ; i ++) {

       			JPanel freespacePanel = new JPanel();
       			JLabel freespace = new JLabel(" ");
       			freespace.setFont(new Font("Dialog", Font.PLAIN, 8));
       			freespacePanel.add(freespace);
       			featValuePanels.add(freespacePanel);
       			curFeaturePanel.add(freespacePanel); 
       			
       			JPanel featureValueAndProbPanel = new JPanel();
       			featureValueAndProbPanel.setLayout(new BoxLayout(featureValueAndProbPanel, BoxLayout.Y_AXIS));
       			
       			JPanel featureValuePanel = new JPanel(new BorderLayout());	
       			JLabel featureValueLabel = new JLabel("Feature value: ");
       			featureValueLabel.setFont(new Font("Dialog", Font.PLAIN, 11));
       			featureValuePanel.add(featureValueLabel, BorderLayout.WEST);
       			JTextField featureValue = new JTextField(6);
       			featureValue.setText("featvalue"+(i+1));
       			featureValue.setName("featvalue");
       			featureValue.setFont(new Font("Dialog", Font.PLAIN, 11));
       			featureValueLabel.setLabelFor(featureValue);
       			featureValuePanel.add(featureValue, BorderLayout.EAST);
       			featureValueAndProbPanel.add(featureValuePanel);
       			
       			JPanel featureProbPanel = new JPanel(new BorderLayout());
       			JLabel featureProbLabel = new JLabel("Prob ( feat value | exists, obs ): ");
       			featureProbLabel.setFont(new Font("Dialog", Font.PLAIN, 11));
       			featureProbPanel.add(featureProbLabel, BorderLayout.WEST);
       			JTextField featureProb = new JTextField(6);
       			featureProb.setText("1.0");
       			featureProb.setName("featprob");
       			featureProb.setFont(new Font("Dialog", Font.PLAIN, 11));
       			featureProbLabel.setLabelFor(featureProb);
       			featureProbPanel.add(featureProb, BorderLayout.EAST);
       			featureValueAndProbPanel.add(featureProbPanel);
       			
       			
       			curFeaturePanel.add(featureValueAndProbPanel);
       			
       			TitledBorder border = BorderFactory.createTitledBorder("Alternative feature value [" + (i+1) + "]");
       			border.setTitleFont(new Font("Dialog", Font.ITALIC, 11));
       			featureValueAndProbPanel.setBorder(BorderFactory.createCompoundBorder(border, BorderFactory.createEmptyBorder(1,1,1,1)));
       			featureValueAndProbPanel.setName("featvalueprob"+i);
       			featValuePanels.add(featureValueAndProbPanel);
       		}
       	}
       	
    	
		insertProxyWindow.proxyPanel.remove(insertProxyWindow.buttonPanel);
       	insertProxyWindow.proxyPanel.add(insertProxyWindow.buttonPanel);


       	insertProxyWindow.validate();
       	insertProxyWindow.repaint();
       	

	}


	private void log(String s) {
		if (LOGGING)
		logger.debug("[ComboBoxListener]" + s);
	}
}
