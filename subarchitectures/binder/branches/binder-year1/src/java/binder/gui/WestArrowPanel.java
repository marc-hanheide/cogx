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

import javax.swing.AbstractButton;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.plaf.basic.BasicArrowButton;

import binder.components.BinderMonitor;

// TODO: implement arrow buttons to navigate in the ranked union configurations

public class WestArrowPanel extends JPanel{

	
	JPanel arrowPanel;

	JPanel buttonPanel;
	
	ComboxBoxListener cbListener;
//	InsertButtonListener buttonListener;
	
	BinderMonitor bm;
	
	static JLabel introLabelf;
	
	public static enum DIRECTION {UP, DOWN};

	int currentRank = 1;

	public WestArrowPanel(BinderMonitor bm) {
		super(new BorderLayout());
		this.bm = bm;
		setLayout(new FlowLayout());
		setMinimumSize(new Dimension(400, 400));

	    add(createArrowPanel());
	    setVisible(true);
	}
	
	
	public int getCurrentRank() {
		return currentRank;
	}
	
	public void setCurrentRank(int rank) {
		currentRank = rank;
	}
	
	private JPanel createArrowPanel() {
		
		arrowPanel = new JPanel();
		arrowPanel.setLayout(new BoxLayout(arrowPanel, BoxLayout.Y_AXIS));
		
		buttonPanel = new JPanel(new BorderLayout());
		
		JLabel introLabel0 = new JLabel("  ");
		JLabel introLabel0b = new JLabel("  Use the arrows to ");
		JLabel introLabel1 = new JLabel("  navigate in the  ");
		JLabel introLabel2 = new JLabel("  alternative union  ");
		JLabel introLabel3 = new JLabel("  configurations:  ");
		JLabel introLabel4 = new JLabel("  ");
		arrowPanel.add(introLabel0, BorderLayout.NORTH);	
		arrowPanel.add(introLabel0b, BorderLayout.NORTH);
		arrowPanel.add(introLabel1, BorderLayout.NORTH);
		arrowPanel.add(introLabel2, BorderLayout.SOUTH);
		arrowPanel.add(introLabel3, BorderLayout.SOUTH);
		arrowPanel.add(introLabel4, BorderLayout.SOUTH);

		JButton upButton = new BasicArrowButton(BasicArrowButton.NORTH);
		upButton.setName("upbutton");
		upButton.setToolTipText("Go to more likely configurations");
		buttonPanel.add(upButton, BorderLayout.NORTH);
	
		upButton.addActionListener(new ChangeConfigListener(bm, this, DIRECTION.UP));

		JButton downButton = new BasicArrowButton(BasicArrowButton.SOUTH);
		downButton.setName("downbutton");
		downButton.setToolTipText("Go to less likely configurations");
		buttonPanel.add(downButton, BorderLayout.SOUTH);
		
		buttonPanel.add(new JLabel(" "));
		
		arrowPanel.add(buttonPanel);
		
		JLabel introLabela = new JLabel("  ");
		JLabel introLabelaa = new JLabel("  ");
		JLabel introLabelaaa = new JLabel("  ");
		JLabel introLabelb = new JLabel("  ");
		JLabel introLabelc = new JLabel("  ");
		JLabel introLabeld = new JLabel("  Current number of  ");
		JLabel introLabele = new JLabel("  alternative union  ");
		int number = 0;
		if (bm.alternativeConfigs != null) {
			number = bm.alternativeConfigs.alterconfigs.length;
		}
		introLabelf = new JLabel("  configurations: " + number);
		
		arrowPanel.add(introLabela, BorderLayout.NORTH);	
		arrowPanel.add(introLabelaa, BorderLayout.NORTH);
		arrowPanel.add(introLabelaaa, BorderLayout.NORTH);
		arrowPanel.add(introLabelb, BorderLayout.NORTH);
		arrowPanel.add(introLabelc, BorderLayout.NORTH);
		arrowPanel.add(introLabeld, BorderLayout.NORTH);
		arrowPanel.add(introLabele, BorderLayout.NORTH);
		arrowPanel.add(introLabelf, BorderLayout.NORTH);

		
		downButton.addActionListener(new ChangeConfigListener(bm, this, DIRECTION.DOWN));
		
		return arrowPanel;
	}
	
	protected static ImageIcon createImageIcon(String path) {
	    return new ImageIcon(path);
	}
	
	public void resetNumberOfConfigurations (int newNumber) {
		arrowPanel.remove(introLabelf);
		introLabelf = new JLabel("  configurations: " + newNumber);
		arrowPanel.add(introLabelf, BorderLayout.NORTH);
	}
}
