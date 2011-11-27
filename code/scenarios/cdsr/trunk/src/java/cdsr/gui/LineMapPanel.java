package cdsr.gui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Line2D;
import java.util.ArrayList;

import javax.swing.JPanel;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;

public class LineMapPanel extends JPanel {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private Room m_room;

	private ArrayList<SensedObject> m_objects;

	private final static int DEFAULT_PIXELS_PER_METRE = 20;

	// private final Logger m_logger = ComponentLogger
	// .getLogger(LineMapPanel.class);

	private final int m_pixelsPerMetre;

	public LineMapPanel() {
		this(DEFAULT_PIXELS_PER_METRE);
	}

	public LineMapPanel(int _pixelsPerMetre) {
		m_pixelsPerMetre = _pixelsPerMetre;
	}

	public void updateRoom(Room _room) {
		m_room = _room;
		repaint();
	}

	public void updateObjects(ArrayList<SensedObject> _objects) {
		m_objects = _objects;
		repaint();
	}

	/**
	 * 
	 * Translate a line to GUI position by scaling and translating. Assumes an
	 * identical transform in x and y.
	 * 
	 * @param _lineInWorldCoords
	 * @return
	 */
	private final Line2D.Double toPixels(Line2D.Double _lineInWorldCoords) {
		return new Line2D.Double(toPixels(_lineInWorldCoords.x1),
				toPixels(_lineInWorldCoords.y1),
				toPixels(_lineInWorldCoords.x2),
				toPixels(_lineInWorldCoords.y2));
	}

	/**
	 * Translate a point to GUI position by scaling and translating. Assumes an
	 * identical transform in x and y.
	 * 
	 * @param _worldPoint
	 * @return
	 */
	private final double toPixels(double _worldPoint) {
		return (_worldPoint * m_pixelsPerMetre) + (getWidth() / 2);
	}

	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
				RenderingHints.VALUE_ANTIALIAS_ON);

		if (m_room != null) {
			g2.setPaint(Color.black);
			for (Line2D.Double segment : m_room) {
				g2.draw(toPixels(segment));
			}
		}

		if (m_objects != null) {
			g2.setPaint(Color.blue);
			for (SensedObject object : m_objects) {
				for (Line2D.Double segment : object) {
					g2.draw(toPixels(segment));
				}
			}
		}

	}

}
