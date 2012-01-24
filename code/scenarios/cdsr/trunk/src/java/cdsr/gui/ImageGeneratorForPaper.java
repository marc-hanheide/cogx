package cdsr.gui;

import java.awt.Color;
import java.awt.geom.Line2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.CDSR;
import cdsr.objects.ProblemSet;
import cdsr.util.Pair;

public abstract class ImageGeneratorForPaper {

	/**
	 * @param args
	 * @throws IOException
	 * @throws ClassNotFoundException
	 */
	public static void main(String[] args) throws IOException,
			ClassNotFoundException {
		if (args.length != 1) {
			System.out
					.println("Only one argument allowed/required, path to image spec file.");
			return;
		}

		LineMapPainter.FILL_CDSRS = true;
		
		File specFile = new File(args[0]);
		BufferedReader reader = new BufferedReader(new FileReader(specFile));

		String[] lines = new String[3];
		int count = 0;
		while (reader.ready() && count < 3) {
			String line = reader.readLine();
			if (!line.startsWith("#")) {
				lines[count++] = line;
				System.out.println(line);
			}
		}
		reader.close();

		ArrayList<Pair<String, Color>> baseData = new ArrayList<Pair<String, Color>>(
				1);
		// base 
		baseData.add(new Pair<String, Color>(lines[0], Color.RED));
		imageWithCDSRs(baseData, "base-image.png");

		ArrayList<Pair<String, Color>> targetData = new ArrayList<Pair<String, Color>>(
				2);
		// target
		targetData.add(new Pair<String, Color>(lines[1], Color.BLUE));
		// inferred
		targetData.add(new Pair<String, Color>(lines[2], Color.GREEN));
		imageWithCDSRs(targetData, "target-image.png");
	}

	// generate base image with annotated CDSR
	static void imageWithCDSRs(List<Pair<String, Color>> _data,
			String _outFile) throws IOException, ClassNotFoundException {

		String firstData = _data.get(0).m_first;

		int firstSpace = firstData.indexOf(' ');
		String imageFilename = firstData.substring(0, firstSpace);
		String dataFilename = GroundTruthScaler.dataFileForImage(imageFilename);

		System.out.println(imageFilename);

		ProblemSet ps = CDSRMarshaller.loadProblemSet(dataFilename);

		int pixelsPerMetre = LineMapImageGenerator.DEFAULT_PIXELS_PER_METRE;
		int imageMargin = LineMapImageGenerator.DEFAULT_IMAGE_MARGIN;

		LineMapImageGenerator imageGen = new LineMapImageGenerator(ps,
				pixelsPerMetre, imageMargin);

		for (Pair<String, Color> pair : _data) {
			firstSpace = pair.m_first.indexOf(' ');
			String groundTruthData = pair.m_first.substring(firstSpace + 1);
			System.out.println(groundTruthData);
			CDSR baseRegion = createCDSRFromDataString(groundTruthData);
			imageGen.addCDSR(baseRegion, pair.m_second);
		}
		imageGen.generateImage(_outFile);
	}

	private static CDSR createCDSRFromDataString(String _data) {
		// WARNING this will fail to process negative numbers sensibly
		String[] split = _data.split("[ ()]+");
		ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>();

		int lastSuccess = 0;
		for (int i = 0; i < split.length - 3; i += 4) {
			try {

				lines.add(new Line2D.Double(Double.parseDouble(split[i]),
						Double.parseDouble(split[i + 1]), Double
								.parseDouble(split[i + 2]), Double
								.parseDouble(split[i + 3])));
				lastSuccess = (i + 3);
			} catch (NumberFormatException e) {
				// HACK this not really a good way of solving this problem
			}
		}

		String type = "unknown";
		// Check whether there is a type on the end
		if (_data.contains("-")) {
			StringBuilder typeBuilder = new StringBuilder();
			for (int i = lastSuccess + 3; i < split.length - 1; i++) {
				typeBuilder.append(split[i]);
				typeBuilder.append('_');
			}
			typeBuilder.append(split[split.length - 1]);

			type = GroundTruthScaler
					.translateToStandard(typeBuilder.toString());
		}

		return new CDSR(lines, type);

	}

}
