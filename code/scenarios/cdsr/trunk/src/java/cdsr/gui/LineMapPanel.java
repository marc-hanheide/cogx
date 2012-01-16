package cdsr.gui;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;

import javax.swing.JPanel;

import cdsr.objects.CDSR;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;

public class LineMapPanel extends JPanel {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private final LineMapPainter m_painter;

	public LineMapPanel() {
		m_painter = new LineMapPainter();
	}

	public LineMapPanel(int _pixelsPerMetre) {
		m_painter = new LineMapPainter(_pixelsPerMetre);
	}

	public void updateRoom(Room _room) {
		m_painter.updateRoom(_room);
		repaint();
	}

	public void updateObjects(ArrayList<SensedObject> _objects) {
		m_painter.updateObjects(_objects);
		repaint();
	}
	

	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		m_painter.setWidth(getWidth());
		m_painter.setHeight(getHeight());
		m_painter.paintLineMap((Graphics2D)g);
	}

	public void addCDSR(CDSR _region) {
		m_painter.addCDSR(_region);
	}

	public void addPoint(Double _worldPoint) {
		m_painter.addPoint(_worldPoint);
	}

}
