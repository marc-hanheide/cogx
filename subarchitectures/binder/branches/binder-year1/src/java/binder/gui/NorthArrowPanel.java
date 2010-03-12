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

// TODO: implement arrow buttons to navigate in binder history

public class NorthArrowPanel extends JPanel{

	
	JPanel proxyPanel;

	JPanel buttonPanel;
	
	ComboxBoxListener cbListener;
//	InsertButtonListener buttonListener;
	
	BinderMonitor bm;

	public NorthArrowPanel(BinderMonitor bm) {
		super(new BorderLayout());
		this.bm = bm;
		setLayout(new FlowLayout());
		setMinimumSize(new Dimension(400, 100));

	    add(createArrowPanel());
	    setVisible(true);
	}
		
	private JPanel createArrowPanel() {
		
		proxyPanel = new JPanel();
		proxyPanel.setLayout(new BoxLayout(proxyPanel, BoxLayout.Y_AXIS));
		
		return proxyPanel;
	}
	
}
