package binder.gui;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.WindowConstants;

import binder.components.BinderMonitor;


public class InsertProxyWindow extends JFrame {

	BinderMonitor bm;

	JPanel buttonPanel;
	
	JPanel proxyPanel;
	
	JScrollPane scrollBar;
	
	ComboxBoxListener cbListener;
	
	public InsertProxyWindow(BinderMonitor bm) {
		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		this.bm = bm;
		setLayout(new FlowLayout());

	//	setMinimumSize(new Dimension(360, 640));
		setSize(new Dimension(360, 680));
		setPreferredSize(new Dimension(360, 680));
		setTitle("Insert new Proxy");
		setResizable(true);
	
		JPanel panel = createNewInsertProxyPanel();
	
		scrollBar=new JScrollPane
		(panel, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);  
		getContentPane().add(scrollBar);

	    setVisible(true);
	    validate();
	}
	
	static int proxyCount =1;
	
	
	// TODO: refactor and test this proxy control panel, and add constraints to disallow probabilities > 1
	
	private JPanel createNewInsertProxyPanel() {
		
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
                                BorderFactory.createTitledBorder("Insert a new proxy in the WM"),
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
		sendbutton.addActionListener(new InsertButtonListener(this, bm));
		
		return proxyPanel;
	}
}
