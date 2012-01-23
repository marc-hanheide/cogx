package cdsr.gui;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.CDSR;
import cdsr.objects.ProblemSet;

public class GroundTruthScaler {

	private final int m_pixelsPerMetre;
	private final int m_imageHeight;
	private final double m_xOriginPixels;
	private final double m_yOriginPixels;
	private final ArrayList<Line2D.Double> m_lines = new ArrayList<Line2D.Double>();
	private Double m_position;
	private String m_type;
	private static HashMap<String, String> m_typeTranslation;
	
	static {
		m_typeTranslation = new HashMap<String, String>();
		m_typeTranslation.put("the_front_of_the_room", "front");
		m_typeTranslation.put("the_back_of_the_room", "back");
		m_typeTranslation.put("the_front_rows", "front_rows");
		m_typeTranslation.put("the_back_rows", "back_rows");
		m_typeTranslation.put("the_kitchen", "kitchen");
		m_typeTranslation.put("the_office", "office");
		m_typeTranslation.put("the_living_room", "living_room");
	}
	
	
	
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

		// WARNING this will fail to process negative numbers sensibly
		String[] split = _data.split("[ ()-]+");

		// can't assume anythign really given Kate's format
		// assert (split.length % 2 == 1);

		GroundTruthScaler scaler = new GroundTruthScaler(_pixelsPerMetre,
				_imageHeight, _xOrigin, _yOrigin);

		int lastSuccess = 0;
		for (int i = 0; i < split.length - 3; i += 4) {
			try {
				scaler.addLine(Integer.parseInt(split[i]),
						Integer.parseInt(split[i + 1]),
						Integer.parseInt(split[i + 2]),
						Integer.parseInt(split[i + 3]));
				lastSuccess = (i + 3);
			} catch (NumberFormatException e) {
				// HACK this not really a good way of solving this problem
			}
		}

		scaler.setPosition(Integer.parseInt(split[lastSuccess + 1]),
				Integer.parseInt(split[lastSuccess + 2]));

		
		//Check whether there is a type on the end
		if (_data.contains("-")) {
			StringBuilder type = new StringBuilder();
			for (int i = lastSuccess + 3; i < split.length - 1; i++) {
				type.append(split[i]);
				type.append('_');
			}
			type.append(split[split.length - 1]);
			
			
			
			scaler.setType(translateToStandard(type.toString()));
		}
		else {
			scaler.setType("unknown");
		}
		return scaler;
	}

	static String translateToStandard(String _type) {
		String trans = m_typeTranslation.get(_type);
		if(trans != null) {
			return trans;
		}
		else {
			return _type;
		}
	}

	private void setType(String _type) {
		m_type = _type;
	}

	public void setPosition(int _x, int _y) {
		m_position = toWorldPoint(_x, _y);
	}

	private void addLine(int _xStart, int _yStart, int _xEnd, int _yEnd) {
		m_lines.add(toWorldLine(_xStart, _yStart, _xEnd, _yEnd));
	}

	public String getType() {
		return m_type;
	}

	public CDSR getCDSR() {
		return new CDSR(m_lines, getType());
	}

	public Point2D.Double getPosition() {
		return m_position;
	}

	public ArrayList<Line2D.Double> getLines() {
		return m_lines;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Line2D.Double line : m_lines) {
			sb.append(line.x1);
			sb.append(' ');
			sb.append(line.y1);
			sb.append(' ');
			sb.append(line.x2);
			sb.append(' ');
			sb.append(line.y2);
			sb.append(' ');
		}
		sb.append('(');
		sb.append(m_position.x);
		sb.append(' ');
		sb.append(m_position.y);
		sb.append(')');
		sb.append(' ');
		sb.append(m_type);
		return sb.toString();
	}

	public static void main(String[] args) throws IOException,
			ClassNotFoundException {

		if (args.length != 1) {
			System.out
					.println("Only one argument allowed/required, path to ground truth file.");
			return;
		}

		File truthFile = new File(args[0]);
		File outFile = new File("translated-" + args[0]);
		BufferedReader reader = new BufferedReader(new FileReader(truthFile));
		BufferedWriter writer = new BufferedWriter(new FileWriter(outFile));

		while (reader.ready()) {
			String line = reader.readLine();
			int firstSpace = line.indexOf(' ');
			String imageFilename = line.substring(0, firstSpace);
			String dataFilename = dataFileForImage(imageFilename);
			String groundTruthData = line.substring(firstSpace + 1);

			System.out.println(imageFilename);
			System.out.println(groundTruthData);

			ProblemSet ps = CDSRMarshaller.loadProblemSet(dataFilename);

			int pixelsPerMetre = LineMapImageGenerator.DEFAULT_PIXELS_PER_METRE;
			int imageMargin = LineMapImageGenerator.DEFAULT_IMAGE_MARGIN;

			LineMapImageGenerator imageGen = new LineMapImageGenerator(ps,
					pixelsPerMetre, imageMargin);

			GroundTruthScaler scaler = GroundTruthScaler.createFromData(
					pixelsPerMetre, imageGen.getImageHeight(),
					imageGen.getXOriginOffset(), imageGen.getYOriginOffset(),
					groundTruthData);

			writer.write(dataFilename);
			writer.write(" ");
			writer.write(scaler.toString());
			writer.write("\n");

			// for visualisation
			// StandaloneViewer viewer = new StandaloneViewer();
			// viewer.addProblemSet(ps);
			//
			// CDSR cdsr = scaler.getCDSR();
			// viewer.addCDSR(cdsr);
			//
			// // add point at origin
			// // System.out.println("origin");
			// // viewer.addPoint(scaler.toWorldPoint(58, 188));
			// Point2D.Double point = scaler.getPosition();
			// viewer.addPoint(point);
			//
			// System.out.println("CDSR");
			// System.out.println(cdsr);
			//
			// System.out.println("Point");
			// System.out.println(point.x + "," + point.y);
		}

		writer.close();
		reader.close();

	}

	/**
	 * Assumes x/y.png matches data/y.cdsr
	 * 
	 * @param _filename
	 * @return
	 */
	static String dataFileForImage(String _filename) {
		String[] split = _filename.split("[/.]+");
		assert (split.length == 3);
		return "data/" + split[1] + ".cdsr";
	}
}
