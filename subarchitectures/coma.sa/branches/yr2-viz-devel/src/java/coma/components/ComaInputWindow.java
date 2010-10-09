package coma.components;

import Ice.Current;
import cast.CASTException;
import cast.architecture.ManagedComponent;

import javax.swing.*;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import comadata.ComaReasonerInterfacePrx;

import java.util.ArrayList;
import java.util.Map;

public class ComaInputWindow extends ManagedComponent {

	private String lastInputFreeForm = "SELECT ?x WHERE { ?x rdfs:subClassOf owl:Thing. }";
	private String lastInputAllSubclassesOf = "";
	private String lastInputAllInstancesOf = "";
	private String lastInputAllRelatedInstancesOf = "";
	
	private String m_comareasoner_component_name;
	private ComaReasonerInterfacePrx m_comareasoner;

	private JTextArea outputArea;
	private JFrame frame;

	private final static String newline = "\n";
    final static boolean shouldFill = true;
    final static boolean shouldWeightX = true;
    final static boolean RIGHT_TO_LEFT = false;
    
    public void configure(Map<String, String> args) {
		log("configure() called");

		if (args.containsKey("--reasoner-name")) {
			m_comareasoner_component_name=args.get("--reasoner-name");
		}
		
	}

	public void start() {
		if (m_comareasoner_component_name==null) {
			log("No coma reasoner present. Exiting! (Specify the coma reasoner component name using --reasoner-name)");
			System.exit(-1);
		}
	
		// initiate ice server connections
		
		// connection to the coma reasoner
		try {
			if (m_comareasoner_component_name!=null) log("initiating connection to Ice server " + m_comareasoner_component_name);
			if (m_comareasoner_component_name!=null) m_comareasoner = getIceServer(m_comareasoner_component_name, comadata.ComaReasonerInterface.class , comadata.ComaReasonerInterfacePrx.class);
			if (m_comareasoner!=null) log("initiated comareasoner connection");
			else throw new CASTException();
		} catch (CASTException e) {
			e.printStackTrace();
			log("Connection to the coma reasoner Ice server at "+ m_comareasoner_component_name + " failed! Exiting. (Specify the coma reasoner component name using --reasoner-name)");
			System.exit(-1);
		}	
	}

	
	protected void runComponent() {
		super.runComponent();
		
		
//		frame = new JFrame("coma GUI");
//		JButton getABox = new JButton("List ABox contents");
//		JButton getTBox = new JButton("List TBox contents");
//		JButton freeForm = new JButton("Free form SPARQL query...");
//		
//		outputArea = new JTextArea(15, 25);
//        outputArea.setEditable(false);
//        JScrollPane scrollPane = new JScrollPane(outputArea);
//
//		
//		getABox.addActionListener(new ActionListener(){
//			public void actionPerformed(ActionEvent ae){
//				debug("getABox button pressed.");
//				String[] abox = m_comareasoner.getAllInstances("owl:Thing");
//				outputArea.append("Listing ABox contents:" + newline);
//				for (String ins : abox) {
//					log(ins);
//					outputArea.append(ins + newline);
//				}
//				outputArea.append("-----" + newline);
//				//Make sure the new text is visible, even if there
//				//was a selection in the text area.
//				outputArea.setCaretPosition(outputArea.getDocument().getLength());
//			}
//		});
//
//		getTBox.addActionListener(new ActionListener(){
//			public void actionPerformed(ActionEvent ae){
//				debug("getTBox button pressed.");
//				outputArea.append("Listing TBox contents:" + newline);
//				String[] tbox = m_comareasoner.getAllSubconcepts("owl:Thing");
//				for (String con : tbox) {
//					log(con);
//					outputArea.append(con + newline);
//				}
//				outputArea.append("-----" + newline);
//				//Make sure the new text is visible, even if there
//				//was a selection in the text area.
//				outputArea.setCaretPosition(outputArea.getDocument().getLength());
//			}
//		});
//
//		freeForm.addActionListener(new ActionListener(){
//			public void actionPerformed(ActionEvent ae){
//				debug("requested free form input window");
//				String str = JOptionPane.showInputDialog(null, "Enter a SPARQL query: ", lastInputFreeForm); 
////						"SPARQL", 1);
//				if(str != null) {
//					log("Executing SPARQL query: " + str);
//					String result = m_comareasoner.executeSPARQL(str);
//					lastInputFreeForm = str;
//					outputArea.append(result + newline);
//				}
//				else {
//					debug("SPARQL input cancelled!");
//				}
//				
//			}
//		});
//		JPanel panel = new JPanel(new GridBagLayout());
//		frame.add(panel);
//		
//		//Add Components to this panel.
//        GridBagConstraints c = new GridBagConstraints();
//
//        c.fill = GridBagConstraints.BOTH;
//        c.weightx = 1.0;
//        c.weighty = 1.0;
//        panel.add(scrollPane, c);
//
//		
//		panel.add(getABox);
//		panel.add(getTBox);
//		panel.add(freeForm);
//		panel.add(scrollPane);
//
//		frame.setSize(400, 400);
//		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frame.setVisible(true);
		
		createAndShowGUI();
	}
	
	public void addComponentsToPane(Container pane) {
		if (RIGHT_TO_LEFT) {
			pane.setComponentOrientation(ComponentOrientation.RIGHT_TO_LEFT);
		}

		JButton getABox = new JButton("List ABox contents");
		pane.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		if (shouldFill) {
			//natural height, maximum width
			c.fill = GridBagConstraints.HORIZONTAL;
		}

		if (shouldWeightX) {
			c.weightx = 0.5;
		}
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		pane.add(getABox, c);

		JButton getTBox = new JButton("List TBox contents");
		c.fill = GridBagConstraints.HORIZONTAL;
		c.weightx = 0.5;
		c.gridx = 1;
		c.gridy = 0;
		pane.add(getTBox, c);

		JButton freeForm = new JButton("Free form SPARQL query...");
		c.fill = GridBagConstraints.HORIZONTAL;
		c.weightx = 0.5;
		c.gridx = 2;
		c.gridy = 0;
		pane.add(freeForm, c);

		outputArea = new JTextArea(15,30);
        outputArea.setEditable(false);
        outputArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
//        outputArea.setRows(30);
        JScrollPane scrollPane = new JScrollPane(outputArea);
		c.fill = GridBagConstraints.BOTH;
		c.ipady = 40;      //make this component tall
		c.weightx = 1.0;
		c.weighty = 1.0;
		c.gridwidth = 3;
		c.gridx = 0;
		c.gridy = 1;
		pane.add(scrollPane, c);

		JButton clear = new JButton("Clear");
		c.fill = GridBagConstraints.HORIZONTAL;
		c.ipady = 0;       //reset to default
		c.weighty = 0.0;   //request any extra vertical space
		c.weightx = 0.0;
		c.anchor = GridBagConstraints.PAGE_END; //bottom of space
//		c.insets = new Insets(10,0,0,0);  //top padding
		c.gridx = 2;       //aligned with button 2
		c.gridwidth = 1;   //2 columns wide
		c.gridy = 2;       //third row
		pane.add(clear, c);

		
		clear.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ae){
				debug("clear button pressed.");
				outputArea.setText("");
			}
		});

		getABox.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent ae){
				debug("getABox button pressed.");
				String[] abox = m_comareasoner.getAllInstances("owl:Thing");
				outputArea.append("Listing ABox contents:" + newline);
				for (String ins : abox) {
					log(ins);
					outputArea.append(ins + newline);
				}
				outputArea.append("-----" + newline);
				//Make sure the new text is visible, even if there
				//was a selection in the text area.
				outputArea.setCaretPosition(outputArea.getDocument().getLength());
			}
		});

		getTBox.addActionListener(new ActionListener(){
		public void actionPerformed(ActionEvent ae){
			debug("getTBox button pressed.");
			outputArea.append("Listing TBox contents:" + newline);
			String[] tbox = m_comareasoner.getAllSubconcepts("owl:Thing");
			for (String con : tbox) {
				log(con);
				outputArea.append(con + newline);
			}
			outputArea.append("-----" + newline);
			//Make sure the new text is visible, even if there
			//was a selection in the text area.
			outputArea.setCaretPosition(outputArea.getDocument().getLength());
		}
	});

	freeForm.addActionListener(new ActionListener(){
		public void actionPerformed(ActionEvent ae){
			debug("requested free form input window");
			String str = (String) JOptionPane.showInputDialog(null, null, "Enter a SPARQL query: ", JOptionPane.PLAIN_MESSAGE, null, null, lastInputFreeForm);  
//			showInputDialog(null, "Enter a SPARQL query: ", lastInputFreeForm); 
//					"SPARQL", 1);
			if(str != null) {
				log("Executing SPARQL query: " + str);
				String result = m_comareasoner.executeSPARQL(str);
				lastInputFreeForm = str;
				outputArea.append(str + newline);
				outputArea.append(result + newline);
				outputArea.append("-----" + newline);
				outputArea.setCaretPosition(outputArea.getDocument().getLength());
			}
			else {
				debug("SPARQL input cancelled!");
			}
			
		}
	});
	}


	/**
	 * Create the GUI and show it.  For thread safety,
	 * this method should be invoked from the
	 * event-dispatching thread.
	 */
	private void createAndShowGUI() {
		//Create and set up the window.
		JFrame frame = new JFrame("coma GUI");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

		//Set up the content pane.
		addComponentsToPane(frame.getContentPane());

		//Display the window.
		frame.pack();
		frame.setVisible(true);
		
	}



	@Override
	public void destroy(Current arg0) {
		frame.dispose();
		super.destroy(arg0);
	}

}
