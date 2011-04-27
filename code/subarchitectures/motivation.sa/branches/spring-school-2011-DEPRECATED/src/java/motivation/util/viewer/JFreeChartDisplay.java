/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.util.viewer;

import java.awt.Rectangle;
import java.io.IOException;
import java.io.StringWriter;

import org.apache.batik.dom.GenericDOMImplementation;
import org.apache.batik.svggen.SVGGraphics2D;
import org.apache.batik.svggen.SVGGraphics2DIOException;
import org.apache.log4j.Logger;
import org.jfree.chart.JFreeChart;
import org.w3c.dom.DOMImplementation;
import org.w3c.dom.Document;

import si.unilj.fri.cogx.v11n.core.DisplayClient;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class JFreeChartDisplay extends DisplayClient {

	/**
	 * @param name
	 */
	public JFreeChartDisplay(String name) {
		super();
		this.name = name;
	}

	final String name;
	DOMImplementation domImpl = GenericDOMImplementation.getDOMImplementation();
	Logger logger = Logger.getLogger(JFreeChartDisplay.class);

	public void display(JFreeChart chart) {
		Document document = domImpl.createDocument(null, "svg", null);

		// Create an instance of the SVG Generator
		SVGGraphics2D svgGenerator = new SVGGraphics2D(document);

		// draw the chart in the SVG generator
		chart.draw(svgGenerator, new Rectangle(600, 600));

		StringWriter sw = new StringWriter();
		try {
			svgGenerator.stream(sw, true /* use css */);
			sw.flush();
			sw.close();
			this.setObject(name, "motives", sw.toString());
		} catch (SVGGraphics2DIOException e) {
			logger.error(e);
		} catch (IOException e) {
			logger.error(e);
		}

	}
}
