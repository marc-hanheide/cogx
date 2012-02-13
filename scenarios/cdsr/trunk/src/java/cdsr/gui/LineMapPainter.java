package cdsr.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.geom.Ellipse2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.List;

import cdsr.marshall.ProblemSetConverter;
import cdsr.objects.CDSR;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cdsr.util.Pair;

public class LineMapPainter {

	private Room m_room;

	private ArrayList<SensedObject> m_objects;

	private final static int DEFAULT_PIXELS_PER_METRE = 80;
	private final static Color DEFAULT_CDSR_COLOUR = Color.green;

	private int m_width;
	private int m_height;

	// how much to move the origin in from the edge of the image, in pixels
	private int m_xOffset;
	private int m_yOffset;

	// private final Logger m_logger = ComponentLogger
	// .getLogger(LineMapPanel.class);

	private final int m_pixelsPerMetre;

	// TODO make configurable
	private boolean m_drawOrigin = false;
	private boolean m_cleanObjectType = true;
	private boolean m_drawTypes = false;
	public static boolean FILL_CDSRS = true;

	private List<Pair<CDSR, Color>> m_cdsrs;

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
//			g2.setStroke(new BasicStroke(3f));
			g2.setPaint(Color.black);

			for (SensedObject object : m_objects) {

				int objectMaxX = Integer.MAX_VALUE;
				int objectMaxY = Integer.MAX_VALUE;

				for (Line2D.Double segment : object) {
					Line2D.Double segmentInPixels = toPixels(segment);
					g2.draw(segmentInPixels);

					Rectangle bounds = segmentInPixels.getBounds();
					if (bounds.getMinX() < objectMaxX) {
						objectMaxX = bounds.x;
					}
					if (bounds.getMinY() < objectMaxY) {
						objectMaxY = bounds.y;
					}
				}
				if (m_drawTypes) {
					String type = object.getType();
					if (m_cleanObjectType) {
						type = ProblemSetConverter
								.formatObjectTypeForDisplay(type);
					}
					g2.drawString(type, objectMaxX - 5, objectMaxY - 5);
				}
			}
		}

		if (m_cdsrs != null) {
			g2.setStroke(new BasicStroke(4f));

			for (Pair<CDSR, Color> cdsr : m_cdsrs) {

				List<java.awt.geom.Line2D.Double> lines = cdsr.m_first
						.getLines();
				GeneralPath polygon = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
				java.awt.geom.Line2D.Double firstLine = lines.get(0);
				polygon.moveTo(toPixelsX(firstLine.x1), toPixelsY(firstLine.y1));
				for (int i = 0; i < lines.size(); i++) {
					polygon.lineTo(toPixelsX(lines.get(i).x2),
							toPixelsY(lines.get(i).y2));
				}
				polygon.closePath();

				// match colour, but add alpha
				Color originalColour = cdsr.m_second;
				g2.setPaint(originalColour);

				if (FILL_CDSRS) {
					Color transparentColour = new Color(
							originalColour.getRed() / 255f,
							originalColour.getGreen() / 255f,
							originalColour.getBlue() / 255f, 0.1f);
					g2.setPaint(transparentColour);
					g2.fill(polygon);
					g2.setPaint(originalColour.darker().darker().darker());
				}

				g2.draw(polygon);

			}
		}

		if (m_points != null) {

			g2.setPaint(Color.orange);
			for (Point2D.Double pt : m_points) {
				g2.draw(new Ellipse2D.Double(toPixelsX(pt.x) - 4,
						toPixelsY(pt.y) - 4, 8, 8));
			}

		}

		if (m_drawOrigin) {
			drawOrigin(g2);
		}

	}

	public void addCDSR(CDSR _region, Color _color) {
		if (m_cdsrs == null) {
			m_cdsrs = new ArrayList<Pair<CDSR, Color>>();
		}
		m_cdsrs.add(new Pair<CDSR, Color>(_region, _color));
	}

	public void addCDSR(CDSR _region) {
		addCDSR(_region, DEFAULT_CDSR_COLOUR);
	}

	public void addPoint(Double _worldPoint) {
		if (m_points == null) {
			m_points = new ArrayList<Point2D.Double>();
		}
		m_points.add(_worldPoint);
	}

}