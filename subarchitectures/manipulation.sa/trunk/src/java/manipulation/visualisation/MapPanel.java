package manipulation.visualisation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.List;
import java.util.Observable;
import java.util.Observer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.swing.JFrame;
import javax.swing.JPanel;

import manipulation.core.bham.simulationConnector.BhamSimulationConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.MathException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Region;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.ItemIntention;
import manipulation.itemMemory.Item.ItemName;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.strategies.MobileManipulationNew;

import org.apache.log4j.Logger;

/**
 * shows the map and further information like position on it
 * 
 * @author dklotz / ttoenige
 * 
 */
public class MapPanel extends JPanel implements Observer {

	private static final long serialVersionUID = 3347424196567633051L;

	private Logger logger = Logger.getLogger(this.getClass());

	private BufferedImage mapImage;

	private final ScheduledExecutorService executor = Executors
			.newSingleThreadScheduledExecutor();

	/** Used for transforming the image. */
	private AffineTransform imageTransform;
	/** Used for transforming all the things drawn on top of the image. */
	private AffineTransform pointTransform;

	private Graphics2D g2;

	private Manipulator manipulator;

	private Item changedItem = null;

	private Observable observable;

	/**
	 * constructor
	 * 
	 * @param manipulator
	 *            relevant manipulator
	 */
	public MapPanel(Manipulator manipulator) {
		super(new BorderLayout());

		this.manipulator = manipulator;
		this.observable = manipulator.getItemMemory();
		this.setPreferredSize(new Dimension(500, 500));
		observable.addObserver(this);

		executor.scheduleWithFixedDelay(new Runnable() {

			@Override
			public void run() {
				try {
					updateMap();
				} catch (Exception e) {
					// logger.error(e);
				}
				repaint();
			}
		}, 200, 1000, TimeUnit.MILLISECONDS);

		// Create and show a window
		JFrame testFrame = new JFrame("MapPanel");
		testFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		testFrame.getContentPane().setLayout(new BorderLayout());
		testFrame.getContentPane().add(this);
		testFrame.pack();
		testFrame.setVisible(true);

		this.addMouseListener(new MouseAdapter() {

			@Override
			public void mouseClicked(MouseEvent mouseEvent) {
				mouseClickAction(mouseEvent);
			}

		});
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#finalize()
	 */
	@Override
	protected void finalize() throws Throwable {
		super.finalize();
		executor.shutdownNow();
	}

	/**
	 * defines what happen if someone clicks on the map
	 * 
	 * @param mouseEvent
	 *            relevant click event
	 */
	private void mouseClickAction(MouseEvent mouseEvent) {

		Region surrounding;
		Item item;

		Vector2D panelCoordinates = new Vector2D(mouseEvent.getX(), mouseEvent
				.getY());
		Vector2D gridCoordinates = panelCoordinates2Grid(panelCoordinates);
		Vector2D worldCoordinates = manipulator.getMapConnector()
				.getWorldFromIndex(gridCoordinates);

		logger.debug("Click at " + gridCoordinates);
		logger.debug("Click at " + worldCoordinates);

		Vector3D translationData = new Vector3D(worldCoordinates.getX(),
				worldCoordinates.getY(), 0.2);

		Matrix rotationData = new Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);

		if (manipulator.getItemMemory().getItemList().isEmpty()) {

			try {
				item = new Item();
				item.setAttribute(PropertyName.NAME, ItemName.FROSTIES_SMALL);
				item.setAttribute(PropertyName.WORLD_POSITION, translationData);
				item.setAttribute(PropertyName.WORLD_ROTATION, rotationData);
				item.setAttribute(PropertyName.INTENTION,
						ItemIntention.GRASP_ME);
				surrounding = manipulator.getMapAlgorithms()
						.updateSurroundingFromWorldCoordinates(item);

				item.setAttribute(PropertyName.SURROUNDING, surrounding);

				ViewPoints vps = new ViewPoints(manipulator.getMapAlgorithms()
						.generatePotentialViewPoints(item));
				item.setAttribute(PropertyName.VIEW_POINTS, vps);

				item.setAttribute(PropertyName.BEST_VIEW_POINT, manipulator
						.getMapAlgorithms().getBestViewPoint(vps.getPoints()));

				manipulator.getItemMemory().addItemToQueue(item);
				repaint();
			} catch (MathException e) {
				logger.error(e);
			} catch (ItemException e) {
				logger.error(e);
			}
		} else {
			try {

				manipulator.getItemMemory().updatePosition(changedItem,
						translationData, new Matrix(1, 0, 0, 0, 1, 0, 0, 0, 1),
						manipulator, true);

				ViewPoints vps;
				try {
					vps = new ViewPoints(manipulator.getMapAlgorithms()
							.generatePotentialViewPoints(changedItem));
					manipulator.getItemMemory().updateViewPoints(
							changedItem,
							vps,
							manipulator.getMapAlgorithms().getBestViewPoint(
									vps.getPoints()));
				} catch (ItemException e) {
					logger.error(e);
				} catch (MathException e) {
					logger.error(e);
				}

				repaint();

			} catch (InternalMemoryException e) {
				logger.error(e);
			}

		}

	}

	// updateMap();

	private Vector2D panelCoordinates2Grid(Vector2D panelCoordinates) {
		try {
			// We just reverse the transformation from grid into pixel
			// coordinates and use that to transform the coordinates in the
			// other direction.
			AffineTransform reverse = pointTransform.createInverse();
			double[] point = new double[] { panelCoordinates.getX(),
					panelCoordinates.getY() };
			reverse.transform(point, 0, point, 0, 1);
			return new Vector2D((int) point[0], (int) point[1]);
		} catch (NoninvertibleTransformException e) {
			// This should normally never happen!
			logger.warn("Could not invert SLAM map transform.", e);
			throw new IllegalStateException("Could not invert transformation "
					+ "that should be invertible?!");
		}
	}

	private void drawMap() {
		// Draw transformed image
		g2.drawImage(mapImage, imageTransform, this);

		try {
			BasePositionData currentPos = manipulator.getBaseConnector()
					.getCurrentPosition();

			Vector2D mapPoint = manipulator.getMapConnector()
					.getIndexFromWorldCoordinates(currentPos);

			g2.setColor(Color.GREEN);
			double[] mapPointArray = mapPoint.toArray();
			pointTransform.transform(mapPointArray, 0, mapPointArray, 0, 1);
			drawPose((int) mapPointArray[0], (int) mapPointArray[1], currentPos
					.getAngle(), true, "");

		} catch (ExternalMemoryException e) {
			logger.warn(e);
		}

	}

	private void drawPose(int x, int y, double yaw, boolean drawOrientation,
			String description) {

		// Draw a circle centered on the position
		int circleRadius = 5;
		g2.drawOval(x - circleRadius, y - circleRadius, (circleRadius * 2) + 1,
				(circleRadius * 2) + 1);

		// Optionally draw a line from the position in the direction of the
		// orientation
		if (drawOrientation) {
			float lineLength = circleRadius + 3;
			g2.drawLine(x, y, x + (int) (cos(yaw) * lineLength), y
					- (int) (sin(yaw) * lineLength));
		}

		// Optionally draw a description to the top right of the circle
		if (!(description == null || "".equals(description.trim()))) {
			g2.drawString(description, x + circleRadius, y - circleRadius);
		}
	}

	private void drawRegion(Region region) {

		List<Vector2D> innerPoints = region.getInnerPoints();
		g2.setColor(Color.RED);

		// for (Vector2D point : innerPoints) {
		//
		// double[] pointArray = manipulator.getMapConnector()
		// .getIndexFromWorldCoordinates(point).toArray();
		//
		// pointTransform.transform(pointArray, 0, pointArray, 0, 1);
		// Rectangle2D pixelRect = new Rectangle2D.Double(pointArray[0],
		// pointArray[1], 1, 1);
		// g2.draw(pixelRect);
		// }
		//

		// List<BorderPoint> borderPoints = region.getBorderPoints();
		// for (BorderPoint borderPoint : borderPoints) {
		// g2.setColor(Color.PINK);
		// Vector2D mapPoint = manipulator.getMapConnector()
		// .getIndexFromWorldCoordinates(borderPoint.getPoint());
		// double[] pointArray = mapPoint.toArray();
		// pointTransform.transform(pointArray, 0, pointArray, 0, 1);
		// drawPose((int) pointArray[0], (int) pointArray[1], 0, false, "");
		//
		// }

		List<ViewPoint> viewPoints;
		try {
			viewPoints = ((ViewPoints) changedItem
					.getAttribute(PropertyName.VIEW_POINTS)).getPoints();

			g2.setColor(Color.BLUE);

			for (ViewPoint point : viewPoints) {
				double[] pointArray = manipulator.getMapConnector()
						.getIndexFromWorldCoordinates(
								point.getPosition().getPoint()).toArray();

				pointTransform.transform(pointArray, 0, pointArray, 0, 1);
				drawPose((int) pointArray[0], (int) pointArray[1], 0, false, "");
			}
		} catch (ItemException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		List<ViewPoint> graspingPoints;
		graspingPoints = ((BhamSimulationConnector) manipulator
				.getSimulationConnector()).getPoints();

		if (!graspingPoints.isEmpty()) {
			g2.setColor(Color.CYAN);

			for (ViewPoint point : graspingPoints) {
				double[] pointArray = manipulator.getMapConnector()
						.getIndexFromWorldCoordinates(
								point.getPosition().getPoint()).toArray();

				pointTransform.transform(pointArray, 0, pointArray, 0, 1);
				drawPose((int) pointArray[0], (int) pointArray[1], 0, false, "");
			}
		}

		if (changedItem.getAllAtributeKeys().contains(
				PropertyName.ROTATIONAL_VIEWPOINT)) {
			try {
				if (!((ViewPoints) changedItem
						.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT))
						.getPoints().isEmpty()) {
					try {
						List<ViewPoint> rotVP = ((ViewPoints) changedItem
								.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT))
								.getPoints();

						g2.setColor(Color.ORANGE);

						for (ViewPoint viewPoint : rotVP) {
							double[] pointArray = manipulator.getMapConnector()
									.getIndexFromWorldCoordinates(
											viewPoint.getPosition().getPoint())
									.toArray();

							pointTransform.transform(pointArray, 0, pointArray,
									0, 1);
							drawPose((int) pointArray[0], (int) pointArray[1],
									0, false, "");
						}

						ViewPoint bestVP = (ViewPoint) changedItem
								.getAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT);

						g2.setColor(Color.PINK);

						double[] pointArray1 = manipulator.getMapConnector()
								.getIndexFromWorldCoordinates(
										bestVP.getPosition().getPoint())
								.toArray();

						pointTransform.transform(pointArray1, 0, pointArray1,
								0, 1);
						drawPose((int) pointArray1[0], (int) pointArray1[1], 0,
								false, "");

					} catch (ItemException e) {
						logger.error(e);
					}
				}
			} catch (ItemException e) {
				logger.error(e);
			}
		}
	}

	private BufferedImage mapToImage() {

		// Graphics setup, gray level image etc.
		BufferedImage image = new BufferedImage(manipulator.getMapConnector()
				.getMap().getxSize(), manipulator.getMapConnector().getMap()
				.getySize(), BufferedImage.TYPE_BYTE_GRAY);
		Graphics2D g2d = image.createGraphics();

		// Create one pixel for each point in the occupancy grid
		for (int x = 0; x < manipulator.getMapConnector().getMap().getxSize(); x++) {
			for (int y = 0; y < manipulator.getMapConnector().getMap()
					.getySize(); y++) {

				double mapValue = manipulator.getMapConnector()
						.getDataFromIndex(new Vector2D(x, y));

				if (mapValue == 2)
					mapValue = 0.5;

				g2d.setColor(new Color((float) mapValue, (float) mapValue,
						(float) mapValue));

				// Create a 1 pixel rectangle and draw it (mirroring the y
				// coordinate, because of the occupancy grids layout, described
				// at the top).

				Rectangle2D pixelRect = new Rectangle2D.Double(x, (image
						.getHeight() - 1)
						- y, 1, 1);
				g2d.draw(pixelRect);
			}
		}

		return image;

	}

	/**
	 * updates the map, gets new information from the manipulator
	 */
	public void updateMap() {

		mapImage = mapToImage();

		imageTransform = new AffineTransform(getWidth()
				/ (double) manipulator.getMapConnector().getMap().getxSize(),
				0, 0, getHeight()
						/ (double) manipulator.getMapConnector().getMap()
								.getySize(), 0, 0);
		pointTransform = new AffineTransform(imageTransform);
		pointTransform.scale(1, -1);
		pointTransform.translate(0, -(manipulator.getMapConnector().getMap()
				.getySize() - 1));

		repaint();
	}

	public void generateNewMap() {
		try {
			manipulator.getMapConnector().updateMap();
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		updateMap();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see javax.swing.JComponent#paint(java.awt.Graphics)
	 */
	@Override
	public void paint(Graphics g) {
		try {
			g2 = (Graphics2D) g;

			g2.clearRect(0, 0, getWidth(), getHeight());
			if (mapImage != null) {
				drawMap();
			}
			if (changedItem != null) {

				drawRegion(((Region) changedItem
						.getAttribute(PropertyName.SURROUNDING)));

				Vector2D mapPoint = manipulator
						.getMapConnector()
						.getIndexFromWorldCoordinates(
								((Vector3D) changedItem
										.getAttribute(PropertyName.WORLD_POSITION)));

				g2.setColor(Color.PINK);
				double[] mapPointArray = mapPoint.toArray();
				pointTransform.transform(mapPointArray, 0, mapPointArray, 0, 1);
				drawPose((int) mapPointArray[0], (int) mapPointArray[1], 0,
						false, "");

			}
		} catch (ItemException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.util.Observer#update(java.util.Observable, java.lang.Object)
	 */
	@Override
	public void update(Observable observable, Object arg) {

		if (observable instanceof ItemMemory) {
			if (arg instanceof Item) {
				Item argItem = ((Item) arg);
				changedItem = argItem;
			}
			if (arg instanceof Vector3D) {
				try {
					changedItem = manipulator.getItemMemory()
							.getFirstGraspItem();
				} catch (InternalMemoryException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
}
