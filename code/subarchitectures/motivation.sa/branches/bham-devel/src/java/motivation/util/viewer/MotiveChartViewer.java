package motivation.util.viewer;

import java.awt.Rectangle;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import java.io.Writer;
import java.util.Map;

import motivation.slice.Motive;
import motivation.util.WMMotiveView;

import org.apache.batik.dom.GenericDOMImplementation;
import org.apache.batik.svggen.SVGGraphics2D;
import org.apache.batik.svggen.SVGGraphics2DIOException;
import org.jfree.ui.RefineryUtilities;
import org.w3c.dom.DOMImplementation;
import org.w3c.dom.Document;

import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView.ChangeHandler;

public class MotiveChartViewer extends ManagedComponent implements
		ChangeHandler<Motive> {

	/**
	 * @author Marc Hanheide (marc@hanheide.de)
	 * 
	 */
	private enum OperationMode {
		SVG, GUI
	}

	private static final long UPDATE_INTERVALL_MS = 5 * 1000;

	/**
	 * Starting point for the demonstration application.
	 * 
	 * @param args
	 *            ignored.
	 * @throws InterruptedException
	 */
	public static void main(final String[] args) throws InterruptedException {

		final MotiveChartViewer demo = new MotiveChartViewer();
		demo.start();
		Thread.sleep(2000);
		// Thread.sleep(2000);
		// demo.dataset.setValue(10, "First", "Type 7");
		// Thread.sleep(2000);
		// demo.dataset.setValue(20, "Second", "Type 10");

	}

	MotiveFrame guiFrame;

	WMMotiveView motives;
	private OperationMode mode = null;

	private JFreeChartDisplay client = new JFreeChartDisplay("MotiveViewer");

	/**
	 * 
	 */
	public MotiveChartViewer() {
		super();
		motives = WMMotiveView.create(this);
		motives.setHandler(this);
		guiFrame = new MotiveFrame(this.getClass().getSimpleName());

	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newEntry, Motive oldEntry)
			throws CASTException {
		guiFrame.update(wmc, newEntry);

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		client.configureDisplayClient(config);
		String strMode = config.get("--mode");
		if (strMode != null)
			mode = OperationMode.valueOf(strMode);
		if (mode == null)
			mode = OperationMode.GUI;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		if (mode == OperationMode.SVG) {
			client.connectIceClient(this);

			while (isRunning()) {
				client.display(guiFrame.getChart());
				try {
					Thread.sleep(UPDATE_INTERVALL_MS);
				} catch (InterruptedException e) {
					if (!isRunning())
						break;
				}

			}

		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {

		/* check if this is actually started in a cast system */
		if (this.getSubarchitectureID() != null) {
			super.start();
			try {
				motives.start();
			} catch (UnknownSubarchitectureException e) {
				logException(e);
			}

			switch (mode) {
			case SVG:

				break;
			case GUI:

				guiFrame.pack();
				RefineryUtilities.centerFrameOnScreen(guiFrame);
				guiFrame.setVisible(true);
				break;
			}
		} else { // testmode

			DOMImplementation domImpl = GenericDOMImplementation
					.getDOMImplementation();
			Document document = domImpl.createDocument(null, "svg", null);

			// Create an instance of the SVG Generator
			SVGGraphics2D svgGenerator = new SVGGraphics2D(document);

			// draw the chart in the SVG generator
			guiFrame.getChart().draw(svgGenerator, new Rectangle(600, 600));

			// Write svg file
			OutputStream outputStream;
			try {
				outputStream = new FileOutputStream("out.svg");
				Writer out = new OutputStreamWriter(outputStream, "UTF-8");
				svgGenerator.stream(out, true /* use css */);
				outputStream.flush();
				outputStream.close();
			} catch (FileNotFoundException e) {
				logException(e);
			} catch (UnsupportedEncodingException e) {
				logException(e);
			} catch (SVGGraphics2DIOException e) {
				logException(e);
			} catch (IOException e) {
				logException(e);
			}

		}

	}

}