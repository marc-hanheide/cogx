package cdsr.gui;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D.Double;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.CDSR;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;

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

	public void addProblemSet(ProblemSet _ps) {
		updateRoom(_ps.getRoom());
		updateObjects(_ps.getObjects());
	}

	public void updateRoom(Room _room) {
	  m_mapPanel.updateRoom(_room);
	}
	
	public void updateObjects(ArrayList<SensedObject> _objects) {
	  m_mapPanel.updateObjects(_objects);
	}
	
	public void addCDSR(CDSR _region) {
		m_mapPanel.addCDSR(_region);
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
					.println("Only one argument allowed/required, path to .cdsr save file.");
			return;
		}

		ProblemSet ps = CDSRMarshaller.loadProblemSet(args[0]);
		
		StandaloneViewer viewer = new StandaloneViewer();
		viewer.addProblemSet(ps);
	}

	public void addPoint(Double _worldPoint) {
		m_mapPanel.addPoint(_worldPoint);
	}

}
