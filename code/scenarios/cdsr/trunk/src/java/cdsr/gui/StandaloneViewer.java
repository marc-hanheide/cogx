package cdsr.gui;

import java.io.IOException;

import javax.swing.JFrame;

import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;

public class StandaloneViewer extends JFrame {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private final LineMapPanel m_mapPanel;

	public StandaloneViewer() {
		m_mapPanel = new LineMapPanel();
		getContentPane().add(m_mapPanel);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setSize(400, 400);
		setLocation(200, 200);
		setVisible(true);
	}

	private void addProblemSet(ProblemSet _ps) {
		m_mapPanel.updateRoom(_ps.getRoom());
		m_mapPanel.updateObjects(_ps.getObjects());
	}

	/**
	 * @param args
	 * @throws ClassNotFoundException
	 * @throws IOException
	 */
	public static void main(String[] args) throws IOException,
			ClassNotFoundException {
		
		if (args.length != 1) {
			System.out
					.println("Only one arguement allowed/required, path to .cdsr save file.");
			return;
		}

		ProblemSet ps = CDSRMarshaller.loadProblemSet(args[0]);
		
		StandaloneViewer viewer = new StandaloneViewer();
		viewer.addProblemSet(ps);
	}

}