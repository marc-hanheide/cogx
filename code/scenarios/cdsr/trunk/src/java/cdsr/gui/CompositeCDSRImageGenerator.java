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

public abstract class CompositeCDSRImageGenerator {

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

		File specFile = new File(args[0]);
		BufferedReader reader = new BufferedReader(new FileReader(specFile));

		ArrayList<Pair<String, Color>> baseData = new ArrayList<Pair<String, Color>>(
				1);
		
		Color cdsrColor = Color.DARK_GRAY;
		
		while (reader.ready()) {
			String line = reader.readLine();
			if (!line.startsWith("#")) {
				baseData.add(new Pair<String,Color>(line,cdsrColor));
			}
		}
		reader.close();

		ImageGeneratorForPaper.imageWithCDSRs(baseData, "composite-image.png");

	}


}
