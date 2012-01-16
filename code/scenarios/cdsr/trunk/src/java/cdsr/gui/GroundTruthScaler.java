package cdsr.gui;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;

import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.CDSR;
import cdsr.objects.ProblemSet;

public class GroundTruthScaler {

	private final int m_pixelsPerMetre;
	private final int m_imageHeight;
	private final double m_xOriginPixels;
	private final double m_yOriginPixels;
	private final ArrayList<Line2D.Double> m_lines = new ArrayList<Line2D.Double>();

	/**
	 * 
	 * @param _pixelsPerMetre
	 * @param _xOrigin
	 *            x origin measures in pixels from the top left of the image
	 * @param _yOrigin
	 *            y origin measures in pixels from the top left of the image
	 */
	public GroundTruthScaler(int _pixelsPerMetre, int _imageHeight,
			int _xOrigin, int _yOrigin) {
		m_pixelsPerMetre = _pixelsPerMetre;
		m_imageHeight = _imageHeight;
		m_xOriginPixels = _xOrigin;
		m_yOriginPixels = _yOrigin;
	}

	/**
	 * Converts pixel position measured from top left of image into world
	 * coords.
	 * 
	 * @param _x
	 * @param _y
	 * @return
	 */
	private Point2D.Double toWorldPoint(int _x, int _y) {

		double xMetres = (_x - m_xOriginPixels) / m_pixelsPerMetre;
		double yMetres = ((m_imageHeight - _y) - (m_imageHeight - m_yOriginPixels))
				/ m_pixelsPerMetre;

		return new Point2D.Double(xMetres, yMetres);
	}

	private Line2D.Double toWorldLine(int _xStart, int _yStart, int _xEnd,
			int _yEnd) {
		return new Line2D.Double(toWorldPoint(_xStart, _yStart), toWorldPoint(
				_xEnd, _yEnd));
	}

	public static GroundTruthScaler createFromData(int _pixelsPerMetre,
			int _imageHeight, int _xOrigin, int _yOrigin, String _data) {

		String[] split = _data.split("[ ()]+");

		assert (split.length % 4 == 2);

		GroundTruthScaler scaler = new GroundTruthScaler(_pixelsPerMetre,
				_imageHeight, _xOrigin, _yOrigin);

		for (int i = 0; i < split.length - 2; i += 4) {
			scaler.addLine(Integer.parseInt(split[i]),
					Integer.parseInt(split[i + 1]),
					Integer.parseInt(split[i + 2]),
					Integer.parseInt(split[i + 3]));
		}

		return scaler;
	}

	private void addLine(int _xStart, int _yStart, int _xEnd, int _yEnd) {
		m_lines.add(toWorldLine(_xStart, _yStart, _xEnd, _yEnd));
	}

	public CDSR getCDSR() {
		return new CDSR(m_lines);
	}

	public ArrayList<Line2D.Double> getLines() {
		return m_lines;
	}

	/**
	 * @param args
	 * @throws ClassNotFoundException
	 * @throws IOException
	 */
	public static void main(String[] args) throws IOException,
			ClassNotFoundException {

		String data = "110 72 112 274 112 274 29 275 29 275 34 76 34 76 112 73 (45 192)";
		GroundTruthScaler scaler = GroundTruthScaler.createFromData(80, 401,
				58, 188, data);

		if (args.length != 1) {
			System.out
					.println("Only one argument allowed/required, path to .cdsr save file.");
			return;
		}

		ProblemSet ps = CDSRMarshaller.loadProblemSet(args[0]);

		StandaloneViewer viewer = new StandaloneViewer();
		viewer.addProblemSet(ps);

		CDSR cdsr = scaler.getCDSR();
		viewer.addCDSR(cdsr);

		// add point at origin
		// System.out.println("origin");
		// viewer.addPoint(scaler.toWorldPoint(58, 188));
		Point2D.Double point = scaler.toWorldPoint(45, 192);
		viewer.addPoint(point);

		System.out.println("CDSR");
		System.out.println(cdsr);

		System.out.println("Point");
		System.out.println(point.x + "," + point.y);
		
	}

}
