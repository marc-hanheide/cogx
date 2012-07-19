package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;

import javax.swing.JFrame;

import org.apache.commons.io.FileUtils;

import de.dfki.lt.tr.pev.vis.ControlToggleCallback;
import de.dfki.lt.tr.pev.vis.FancySchmanzyDemovisor;
import de.dfki.lt.tr.planverb.generation.Message;
import de.dfki.lt.tr.planverb.generation.ProtoLFMessage;
import de.dfki.lt.tr.planverb.generation.StringMessage;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLHistory;

public class StandaloneHistoryVerbalizer implements Runnable {
	
	/**
	 * main method for running stand-alone PEV
	 * subarchitectures/dialogue.sa/resources/dora-interactive_annotated.txt subarchitectures/dialogue.sa/resources/domain2test.pddl subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt localhost 4321 subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-02_16:02/GBeliefHistory.xml subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-02_16:02/history-1.pddl 1 
	 * subarchitectures/dialogue.sa/resources/dora-interactive_annotated.txt subarchitectures/dialogue.sa/resources/domain2test.pddl subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt localhost 4321 subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-09_16:20/GBeliefHistory.xml subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-09_16:20/history-3.pddl 3 2012-07-02_16:02 2012-07-03_15:14 2012-07-09_13:35 2012-07-09_14:24 2012-07-09_16:20 (hat 1 und 3)
	 * 
	 * other runs: 
	 * 2012-07-02_16:02 
	 * 2012-07-03_15:14 
	 * 2012-07-09_13:35 
	 * 2012-07-09_14:24 
	 * 2012-07-09_16:20 (has histories 1 and 3)
	 * 
	 * 
	 * subarchitectures/dialogue.sa/resources/dora-interactive_annotated.txt
subarchitectures/dialogue.sa/resources/domain2test.pddl
subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml
subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt
localhost
4321
pev-data/t621-UBK_pevmissing/2012-07-14_12:19/ 1

pev-data/t602-empty_map_search_successful/2012-07-12_18:39 1

pev-data/t621-UBK_pevmissing/2012-07-14_12:19/GBeliefHistory.xml
pev-data/t621-UBK_pevmissing/2012-07-14_12:19/history-1.pddl
1

pev-data/t621-UBK_pevwrong/2012-07-14_11:30
pev-data/t621-UBK_pevwrong/2012-07-14_11:30

subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-05_17:47/GBeliefHistory.xml
subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-05_17:47/history-1.pddl
1
	 * 
	 * @param args, see above
	 */
	public static void main(String[] args) throws NumberFormatException, IOException {
		int taskId = Integer.parseInt(args[7]);
		File f = new File(args[6] + "/history-" + taskId + ".pddl");
		String gbhFile = args[6] + "/GBeliefHistory.xml";
		PlanVerbalizer pevMod = new PlanVerbalizer(args[0], args[1], args[2], args[3], args[4], Integer.parseInt(args[5]), gbhFile);
		pevMod.debug_lf_out = true;
	
		StandaloneHistoryVerbalizer standaloneDemo = new StandaloneHistoryVerbalizer(pevMod, f, taskId);
		standaloneDemo.run();
		
	}
	
	private PlanVerbalizer pevModule;
	private FancySchmanzyDemovisor demoGui;
	private boolean startButton = false;
	File f;
	int taskId;
	
	public StandaloneHistoryVerbalizer(PlanVerbalizer pevModule, File f, int taskId) throws IOException  {
		this.f = f;
		this.taskId = taskId;
		this.pevModule = pevModule;
		String historyText = FileUtils.readFileToString(f);
		
		demoGui = new FancySchmanzyDemovisor();
		demoGui.getJFrame().pack();
		demoGui.setPOPlanText(historyText);
		demoGui.getJFrame().setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		demoGui.getJFrame().setVisible(true);
		
		ControlToggleCallback cbk = new ControlToggleCallback() {
			
			public void toggled(Boolean arg0) {
				buttonPressed(arg0);
			}
		};

		demoGui.addControlToggleCallback(cbk);
	}

	private void buttonPressed(Boolean arg0) {
		synchronized(this) {
			startButton = arg0;
			notifyAll();
		}

	}

	@Override
	public void run() {	
		try {
			List<Message> messages = this.pevModule.verbalizeHistoryStepOne(new PDDLHistory(f, taskId));
			
			int i = 1;
			for (Message message : messages) {
				if (message instanceof ProtoLFMessage) {
					demoGui.addMessages(i++ + ") LFMessage: " + ((ProtoLFMessage) message).getProtoLF().toString()+"\n\n");
				} else if (message instanceof StringMessage) {
					demoGui.addMessages(i++ + ") CTMessage: " +((StringMessage) message).getText()+"\n\n");
				}
			}

			i = 1;
			StringBuilder rawTextBuilder = new StringBuilder(messages.size()*10);
			synchronized(this) {
				for (Message message : messages) {

					while (!startButton) {
//						System.out.println("stop");
						try {
							this.wait(1000);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}

//					System.out.println("start");
					String rawRealization = pevModule.realizeMessage(message);
					rawTextBuilder.append(rawRealization);
					demoGui.addRawText(i++ + ") " + rawRealization);

					try {
						this.wait(1000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}

				}
			} 

			String outText = PEVUtils.aggregateStrings(rawTextBuilder.toString());
			demoGui.setFancyText(outText);
			System.out.println(outText);
			
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
			System.exit(-1);
		}
		
		
	}



}
