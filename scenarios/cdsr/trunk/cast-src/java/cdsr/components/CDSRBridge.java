package cdsr.components;

import java.awt.geom.Line2D;
import java.awt.geom.Line2D.Double;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;

import javax.swing.JFrame;

import NavData.LineMap;
import NavData.LineMapSegement;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cdsr.gui.LineMapPanel;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.Room;

public class CDSRBridge extends ManagedComponent {

	private final LineMapPanel m_panel;

	private Room m_cdsrRoom;

	private String m_outputFile;

	public CDSRBridge() {
		m_panel = new LineMapPanel();
	}

	@Override
	protected void start() {

		addChangeFilter(ChangeFilterFactory.createTypeFilter(LineMap.class,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						lineMapUpdated(_wmc.address.id,
								getMemoryEntry(_wmc.address, LineMap.class));
					}
				});

	}

	@Override
	public void configure(Map<String, String> _config) {
		m_outputFile = _config.get("--output");
	}

	private void lineMapUpdated(String _id, LineMap _lineMap) {
		updateRoom(_id, _lineMap);
		m_panel.updateRoom(m_cdsrRoom);
	}

	private void updateRoom(String _id, LineMap _lineMap) {
		// TODO Extract room line segments from room

		if (m_cdsrRoom == null) {
			m_cdsrRoom = new Room(_id, new ArrayList<Line2D.Double>(
					_lineMap.lines.length));
		}

		m_cdsrRoom.getLines().clear();
		toCDSRLines(_lineMap.lines, m_cdsrRoom.getLines());
	}

	private void toCDSRLines(LineMapSegement[] _lines,
			ArrayList<Line2D.Double> _cdsrlines) {
		for (LineMapSegement line : _lines) {
			_cdsrlines.add(new Double(line.start.x, line.start.y, line.end.x,
					line.end.y));
		}
	}

	@Override
	protected void stop() {
		if (m_outputFile != null) {
			try {
				CDSRMarshaller.saveProblemSet(m_outputFile, m_cdsrRoom);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	@Override
	protected void runComponent() {
		JFrame f = new JFrame("CDSR map");
		f.getContentPane().add(m_panel);
		f.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		f.setSize(400, 400);
		f.setLocation(200, 200);
		f.setVisible(true);
	}
}
