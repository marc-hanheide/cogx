package cdsr.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.List;

import cdsr.objects.CDSR;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;

public class LineMapPainter {

	private Room m_room;

	private ArrayList<SensedObject> m_objects;

	private final static int DEFAULT_PIXELS_PER_METRE = 80;

	private int m_width;
	private int m_height;

	// how much to move the origin in from the edge of the image, in pixels
	private int m_xOffset;
	private int m_yOffset;

	// private final Logger m_logger = ComponentLogger
	// .getLogger(LineMapPanel.class);

	private final int m_pixelsPerMetre;

	private boolean m_drawOrigin = false;

	private List<CDSR> m_cdsrs;

	private ArrayList<Point2D.Double> m_points;

	public LineMapPainter() {
		this(DEFAULT_PIXELS_PER_METRE);
	}

	public LineMapPainter(int _pixelsPerMetre, int _xOffset, int _yOffset) {
		m_pixelsPerMetre = _pixelsPerMetre;
		m_xOffset = _xOffset;
		m_yOffset = _yOffset;
	}

	public LineMapPainter(int _pixelsPerMetre) {
		this(_pixelsPerMetre, 3 * _pixelsPerMetre, 3 * _pixelsPerMetre);
	}

	public void updateRoom(Room _room) {
		m_room = _room;
	}

	public void updateObjects(ArrayList<SensedObject> _objects) {
		m_objects = _objects;
	}

	public int getWidth() {
		return m_width;
	}

	public void setWidth(int _width) {
		m_width = _width;
	}

	public int getHeight() {
		return m_height;
	}

	public void setHeight(int _height) {
		m_height = _height;
	}

	private int invertY(int _y) {
		return getHeight() - _y;
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
		return new Line2D.Double(toPixelsX(_lineInWorldCoords.x1),
				toPixelsY(_lineInWorldCoords.y1),
				toPixelsX(_lineInWorldCoords.x2),
				toPixelsY(_lineInWorldCoords.y2));
	}

	/**
	 * Translate a point to GUI position by scaling and translating. Assumes an
	 * identical transform in x and y.
	 * 
	 * @param _worldPoint
	 * @return
	 */
	private final int toPixelsX(double _worldPoint) {
		return (int) (_worldPoint * m_pixelsPerMetre) + m_xOffset;
	}

	private final int toPixelsY(double _worldPoint) {
		return invertY((int) (_worldPoint * m_pixelsPerMetre) + m_yOffset);
	}

	private void drawOrigin(Graphics2D g2) {
		g2.setPaint(Color.red);
		g2.draw(toPixels(new Line2D.Double(0, 0, 1, 0)));
		g2.drawString("x", toPixelsX(1), toPixelsY(0));
		g2.draw(toPixels(new Line2D.Double(0, 0, 0, 1)));
		g2.drawString("y", toPixelsX(0), toPixelsY(1));

	}

	public void paintLineMap(Graphics2D g2) {
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
				RenderingHints.VALUE_ANTIALIAS_ON);

		g2.setPaint(Color.white);
		g2.fillRect(0, 0, getWidth(), getHeight());

		if (m_room != null) {
			g2.setPaint(Color.black);
			for (Line2D.Double segment : m_room) {
				g2.draw(toPixels(segment));
			}
		}

		if (m_objects != null) {
			g2.setStroke(new BasicStroke(3f));
			g2.setPaint(Color.blue);
			for (SensedObject object : m_objects) {
				for (Line2D.Double segment : object) {
					g2.draw(toPixels(segment));
				}
			}
		}

		if (m_cdsrs != null) {
			g2.setStroke(new BasicStroke(2f));
			g2.setPaint(Color.green);

			for (CDSR cdsr : m_cdsrs) {
				for (Line2D.Double segment : cdsr) {
					g2.draw(toPixels(segment));
				}
			}
		}

		if (m_points != null) {

//			g2.setPaint(Color.orange);
			for (Point2D.Double pt : m_points) {
				g2.draw(new Ellipse2D.Double(toPixelsX(pt.x) - 4, toPixelsY(pt.y) - 4, 8, 8));
			}

		}

		if (m_drawOrigin) {
			drawOrigin(g2);
		}

	}

	public void addCDSR(CDSR _region) {
		if (m_cdsrs == null) {
			m_cdsrs = new ArrayList<CDSR>();
		}
		m_cdsrs.add(_region);
	}

	public void addPoint(Double _worldPoint) {
		if (m_points == null) {
			m_points = new ArrayList<Point2D.Double>();
		}
		m_points.add(_worldPoint);
	}

}
