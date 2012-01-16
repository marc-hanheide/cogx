package cdsr.gui;

import java.awt.geom.Line2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;

/**
 * A class to generate an image file from a set of CDSR objects.
 * 
 * @author nah
 * 
 */
public class LineMapImageGenerator {

	private int m_width;
	private int m_height;
	private int m_xOffset;
	private int m_yOffset;

	private final LineMapPainter m_painter;

	public LineMapImageGenerator(ProblemSet _ps, int _pixelsPerMetre,
			int _imageMarginPixel) {

		calculateImageDimensions(_ps.getRoom(), _pixelsPerMetre,
				_imageMarginPixel);

		m_painter = new LineMapPainter(_pixelsPerMetre, m_xOffset, m_yOffset);
		m_painter.setWidth(m_width);
		m_painter.setHeight(m_height);
		m_painter.updateObjects(_ps.getObjects());
		m_painter.updateRoom(_ps.getRoom());
		
		
		System.out.println(_pixelsPerMetre + " pixels/m");
		System.out.println("origin at pixel " + m_xOffset + ", " + (m_height - m_yOffset) + " measured from top left");
	}

	private void calculateImageDimensions(Room _room, int _pixelsPerMetre,
			int _imageMarginPixel) {
		double maxX = Double.MIN_VALUE;
		double minX = Double.MAX_VALUE;
		double maxY = Double.MIN_VALUE;
		double minY = Double.MAX_VALUE;

		for (Line2D.Double line : _room) {
			if (line.x1 > maxX) {
				maxX = line.x1;
			}
			if (line.y1 > maxY) {
				maxY = line.y1;
			}
			if (line.y1 < minY) {
				minY = line.y1;
			}
			if (line.x2 > maxX) {
				maxX = line.x2;
			}
			if (line.y2 > maxY) {
				maxY = line.y2;
			}
			if (line.x2 < minX) {
				minX = line.x2;
			}
			if (line.y2 < minY) {
				minY = line.y2;
			}
		}

		// calculate how much to translate map by to make it fit nicely in the
		// image

		if (minX != 0) {
			m_xOffset = ((int) (-minX * _pixelsPerMetre));
		} else {
			m_xOffset = 0;
		}
		if (minY != 0) {
			m_yOffset = ((int) (-minY * _pixelsPerMetre));
		} else {
			m_yOffset = 0;
		}

		//extend offset by margin
		m_xOffset += _imageMarginPixel;
		m_yOffset += _imageMarginPixel;		
		
		m_width = ((int) ((maxX - minX) * _pixelsPerMetre))
				+ (2 * _imageMarginPixel);
		m_height = ((int) ((maxY - minY) * _pixelsPerMetre))
				+ (2 * _imageMarginPixel);

	}

	private boolean generateImage(String _filename) throws IOException {
		BufferedImage img = new BufferedImage(m_width, m_height,
				BufferedImage.TYPE_INT_RGB);
		m_painter.paintLineMap(img.createGraphics());

		// use the desired file extension to create the writer type
		String fileType = _filename.substring(_filename.lastIndexOf(".") + 1);

		return ImageIO.write(img, fileType, new File(_filename));

	}

	/**
	 * @param args
	 * @throws IOException
	 * @throws ClassNotFoundException
	 */
	public static void main(String[] args) throws IOException,
			ClassNotFoundException {

		if (args.length != 2) {
			System.out
					.println("Only two arguments allowed/required, path to .cdsr save file followed by desired output file");
			return;
		}

		ProblemSet ps = CDSRMarshaller.loadProblemSet(args[0]);
		LineMapImageGenerator generator = new LineMapImageGenerator(ps, 80, 10);

		if (!generator.generateImage(args[1])) {
			System.err.println("image generation failed");
		}
	}

}
