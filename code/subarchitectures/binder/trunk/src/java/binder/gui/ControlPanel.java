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
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import binder.components.BinderMonitor;

// TODO: design a window to control various binder parameters

public class ControlPanel extends JPanel{

	
	JPanel proxyPanel;

	JPanel buttonPanel;
	
	ComboxBoxListener cbListener;
//	InsertButtonListener buttonListener;
	
	BinderMonitor bm;

	public ControlPanel(BinderMonitor bm) {
		super(new BorderLayout());
		this.bm = bm;
		setLayout(new FlowLayout());
		setMinimumSize(new Dimension(400, 400));

	    add(createNewProxyControlPanel());
	    setVisible(true);
	}
	
	static int proxyCount =1;
	
	
	// TODO: refactor and test this proxy control panel, and add constraints to disallow probabilities > 1
	
	private JPanel createNewProxyControlPanel() {
		
		proxyPanel = new JPanel();
		proxyPanel.setLayout(new BoxLayout(proxyPanel, BoxLayout.Y_AXIS));
		
		JPanel proxyPanel1 = new JPanel(new BorderLayout());
		JLabel proxyIDLabel = new JLabel("Proxy ID: ");
		proxyPanel1.add(proxyIDLabel, BorderLayout.WEST);	
		JTextField proxyID = new JTextField(8);
		proxyID.setText("proxyID"+proxyCount);
		proxyCount++;
		proxyID.setName("proxyID");
		proxyIDLabel.setLabelFor(proxyID);
		proxyPanel1.add(proxyID, BorderLayout.EAST);
		proxyPanel.add(proxyPanel1);
		
		JPanel proxyPanel2 = new JPanel(new BorderLayout());
		JLabel subarchLabel = new JLabel("Originating subarchitecture:   ");
		proxyPanel2.add(subarchLabel, BorderLayout.WEST);
		JTextField subarch = new JTextField(8);
		subarch.setText("default");
		subarch.setName("subarch");
		subarchLabel.setLabelFor(subarch);
		proxyPanel2.add(subarch, BorderLayout.EAST);
		proxyPanel.add(proxyPanel2);
		
		JPanel proxyPanel3 = new JPanel(new BorderLayout());
		JLabel existsLabel = new JLabel("Prob ( exists | obs ): ");
		proxyPanel3.add(existsLabel, BorderLayout.WEST);
		JTextField exists = new JTextField(8);
		exists.setText("1.0");
		exists.setName("exists");
		existsLabel.setLabelFor(exists);
		proxyPanel3.add(exists, BorderLayout.EAST);
		proxyPanel.add(proxyPanel3);
		
		JLabel freespace = new JLabel(" ");
		freespace.setFont(new Font("Dialog", Font.PLAIN, 8));
		proxyPanel.add(freespace);
   			
		JPanel proxyPanel4 = new JPanel(new BorderLayout());
		JLabel featuresLabel = new JLabel("Number of features");
		proxyPanel4.add(featuresLabel, BorderLayout.WEST);
		String[] nbFeatures = {"0", "1", "2", "3", "4", "5"};
		JComboBox nbFeaturesBox = new JComboBox(nbFeatures);
		nbFeaturesBox.setName("nbFeatures");
		featuresLabel.setLabelFor(nbFeaturesBox);
		nbFeaturesBox.addActionListener(new ComboxBoxListener(this));
		proxyPanel4.add(nbFeaturesBox, BorderLayout.EAST);
		proxyPanel.add(proxyPanel4);
        
		proxyPanel.setBorder(
                BorderFactory.createCompoundBorder(
                                BorderFactory.createTitledBorder("Insert a new proxy"),
                                BorderFactory.createEmptyBorder(5,5,5,5)));
		
		buttonPanel = new JPanel(new BorderLayout());
		JPanel freespacePanel = new JPanel();
		freespace = new JLabel(" ");
		freespacePanel.add(freespace);
		buttonPanel.add(freespacePanel, BorderLayout.NORTH);
		
		JButton sendbutton = new JButton("Insert new proxy");
		buttonPanel.add(sendbutton, BorderLayout.SOUTH);
		proxyPanel.add(buttonPanel);
		sendbutton.setName("sendbutton");
		sendbutton.addActionListener(new InsertButtonListener(proxyPanel, bm));
		
		return proxyPanel;
	}
	
}
