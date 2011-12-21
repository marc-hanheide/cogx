package cdsr.components;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.swing.JFrame;

import mathlib.Functions;
import NavData.LineMap;
import NavData.LineMapSegement;
import VisionData.Vertex;
import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cdsr.gui.LineMapPanel;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

public class CDSRBridge extends ManagedComponent {

	private final LineMapPanel m_panel;

	private Room m_cdsrRoom;

	private String m_outputFile;

	private HashMap<String, SensedObject> m_sensedObjects;

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

		WorkingMemoryChangeReceiver voReceiver = new WorkingMemoryChangeReceiver() {

			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {

				VisualObject visualObject = getMemoryEntry(_wmc.address,
						VisualObject.class);

				visualObjectUpdated(_wmc.address.id, visualObject);
			}
		};

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD), voReceiver);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE),
				voReceiver);

	}

	private void visualObjectUpdated(String _id, VisualObject _visualObject) {

		if (_visualObject.identDistrib[0] >= 0.5) {
			log("Detected " + _visualObject.identLabels[0] + " at " + _id);
			storeObject(_id, _visualObject);
		} else {
			log("Did not detect " + _visualObject.identLabels[0] + " at " + _id);
		}

	}

	private SensedObject toCDSRObject(String _id, VisualObject _visualObject) {
		// println("pose");
		// println(_visualObject.pose.pos.x + " " + _visualObject.pose.pos.y +
		// " "
		// + _visualObject.pose.pos.z);

		// println("rotation");
		// println(_visualObject.pose.rot.m00 + " " + _visualObject.pose.rot.m10
		// + " " + _visualObject.pose.rot.m20);
		// println(_visualObject.pose.rot.m01 + " " + _visualObject.pose.rot.m11
		// + " " + _visualObject.pose.rot.m22);
		// println(_visualObject.pose.rot.m02 + " " + _visualObject.pose.rot.m12
		// + " " + _visualObject.pose.rot.m22);

		// println("faces");
		// Face[] faces = _visualObject.model.faces;
		// for (Face face : faces) {
		// println(Arrays.toString(face.vertices));
		// }

		// println("vertices");
		Vertex[] vertices = _visualObject.model.vertices;
		ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>(
				vertices.length / 2);
		for (int i = 0; i < vertices.length; i += 2) {

			Line2D.Double line = toCDSRLine(vertices[i], vertices[i + 1],
					_visualObject.pose);

			if (line != null) {
				log("line: " + line.x1 + "," + line.y1 + "," + line.x2 + ","
						+ line.y2);

				// doesn't seem to happen yet
				// if (lines.contains(line)) {
				// println("seen before");
				// }

				lines.add(line);
			}
		}

		return new SensedObject(_id, lines, _visualObject.identLabels[0]);
	}

	private Point2D.Double toWorldPoint(Vertex _vertex, Pose3 _pose) {
		// Transform given vertex to be in world coords relative to object pose
		Vector3 transformedPose = Functions.transform(_pose, _vertex.pos);
		return new Point2D.Double(transformedPose.x, transformedPose.y);
	}

	private Line2D.Double toCDSRLine(Vertex _vertex1, Vertex _vertex2,
			Pose3 _pose) {

		Point2D.Double pt1 = toWorldPoint(_vertex1, _pose);
		Point2D.Double pt2 = toWorldPoint(_vertex2, _pose);
		Line2D.Double line = null;
		// A line in 3D may become a point in 2D
		if (!pt1.equals(pt2)) {
			line = new Line2D.Double(pt1, pt2);
		} else {
			// println("no line, returning null");
		}
		return line;
	}

	private void storeObject(String _id, VisualObject _visualObject) {
		if (m_sensedObjects == null) {
			m_sensedObjects = new HashMap<String, SensedObject>();
		}

		// this assume objects don't get updated on WM in meaningful ways after
		// they've been successfully detected
		if (!m_sensedObjects.containsKey(_id)) {
			m_sensedObjects.put(_id, toCDSRObject(_id, _visualObject));
			m_panel.updateObjects(new ArrayList<SensedObject>(m_sensedObjects
					.values()));
		}
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
			_cdsrlines.add(new Line2D.Double(line.start.x, line.start.y,
					line.end.x, line.end.y));
		}
	}

	@Override
	protected void stop() {
		if (m_outputFile != null) {
			try {
				CDSRMarshaller.saveProblemSet(m_outputFile, m_cdsrRoom,
						new ArrayList<SensedObject>(m_sensedObjects.values()));
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
