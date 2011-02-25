package manipulation.core.bham.simulationConnector;

import golem.tinyice.ArmDesc;
import golem.tinyice.ArmPrx;
import golem.tinyice.ArmPrxHelper;
import golem.tinyice.BoxShapeDesc;
import golem.tinyice.ExTiny;
import golem.tinyice.JointPrx;
import golem.tinyice.KatanaArmDesc;
import golem.tinyice.KatanaArmPrxHelper;
import golem.tinyice.Mat33;
import golem.tinyice.Mat34;
import golem.tinyice.PlaneShapeDesc;
import golem.tinyice.RigidBodyDesc;
import golem.tinyice.RigidBodyPrx;
import golem.tinyice.RigidBodyPrxHelper;
import golem.tinyice.ShapeDesc;
import golem.tinyice.TinyPrx;
import golem.tinyice.TinyPrxHelper;
import golem.tinyice.Vec3;

import java.util.Vector;

import manipulation.core.bham.converter.BhamConverter;
import manipulation.core.bham.simulationConnector.initObjects.ArmDescI;
import manipulation.core.bham.simulationConnector.initObjects.BoxShapeDescI;
import manipulation.core.bham.simulationConnector.initObjects.KatanaArmDescI;
import manipulation.core.bham.simulationConnector.initObjects.PlaneShapeDescI;
import manipulation.core.bham.simulationConnector.initObjects.RigidBodyDescI;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.simulationConnector.SimulationConnector;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

/**
 * represents a connector to the GOLEM virtual scene (Birmingham / CogX
 * environment)
 * 
 * @author ttoenige
 * 
 */
public class BhamSimulationConnector implements SimulationConnector {
	private static final double BASEHIGHT = 0.215;
	private static final double BASELENGTH = 0.51;
	private static final double BASEWIDTH = 0.38;

	private static final double STICKHIGHT = 1;
	private static final double STICKLENGTH = 0.05;
	private static final double STICKWIDTH = 0.05;

	private static final double FINGERLENGTH = 0.15;
	private static final double FINGERDIAM = 0.005;
	private static final double GRIPPERLENGTH = 0.08;

	// private static final double fingerTipRadius = 0.015;

	private Logger logger = Logger.getLogger(this.getClass());

	private Matrix initArmRotation = MathOperation
			.getRotationAroundZ(-(Math.PI / 2));

	private Vector<RigidBodyPrx> obstacles;
	private Vector<RigidBodyPrx> tables;
	public TinyPrx tinyInterface;
	private RigidBodyPrx robot = null;
	private ArmPrx arm;

	private Manipulator manipulator;

	private Thread posThread;
	private Thread itemThread;

	private Vector<ViewPoint> points = new Vector<ViewPoint>();

	/**
	 * constructor of the connector to the GOLEM virtual scene
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public BhamSimulationConnector(Manipulator manipulator) {
		obstacles = new Vector<RigidBodyPrx>();
		tables = new Vector<RigidBodyPrx>();
		this.manipulator = manipulator;

		posThread = new Thread(new UpdatePositionRunnable(this, manipulator));
		itemThread = new Thread(new UpdateItemPositionRunnable(this,
				manipulator));

		Ice.Communicator ic = null;
		ic = Ice.Util.initialize();
		// TODO KONSTANTE
		Ice.ObjectPrx init = ic.stringToProxy("GolemTiny:default -p 8172");
		tinyInterface = TinyPrxHelper.checkedCast(init);
		if (tinyInterface == null) {
			logger.error("invalid golem Tiny connection");
		}

		try {
			createGroundPlane();
			BasePositionData position = new BasePositionData(
					new Vector2D(0, 0), 0);
			robot = updateRobot(position);
			arm = createArm(manipulator.getConfiguration().isSimulation());
		} catch (Exception e) {
			logger.error(e);
		}

	}

	private RigidBodyPrx createGroundPlane() throws Exception {
		RigidBodyDesc pGroundPlaneDesc = new RigidBodyDescI();
		PlaneShapeDesc pGroundPlaneShapeDesc = new PlaneShapeDescI();
		pGroundPlaneDesc.shapes[0] = pGroundPlaneShapeDesc;
		return RigidBodyPrxHelper.checkedCast(tinyInterface
				.createActor(pGroundPlaneDesc));
	}

	private ArmPrx createArm(boolean simulation) {
		Vec3 globalArmPosePos = new Vec3(0, 0, BASEHIGHT);
		Mat33 globalArmPoseRot = BhamConverter
				.convMatrixToGolem(initArmRotation);
		ArmPrx arm = null;
		logger.debug("Creating Arm...");

		if (simulation) {
			ArmDesc pArmDesc = new ArmDescI();
			pArmDesc.path = "GolemDeviceKatana300Sim";
			pArmDesc.globalPose.p = globalArmPosePos;
			pArmDesc.globalPose.R = globalArmPoseRot;

			try {
				arm = ArmPrxHelper.checkedCast(tinyInterface
						.createActor(pArmDesc));
			} catch (Exception e) {
				logger.error(e);
			}

		} else {
			KatanaArmDesc pArmDesc = new KatanaArmDescI();
			pArmDesc.path = "GolemDeviceKatana300";
			pArmDesc.bGripper = true;
			pArmDesc.globalPose.p = globalArmPosePos;
			pArmDesc.globalPose.R = globalArmPoseRot;
			try {
				arm = KatanaArmPrxHelper.checkedCast(tinyInterface
						.createActor(pArmDesc));
			} catch (Exception e) {
				logger.error(e);
			}

		}

		JointPrx pEffector = arm.getJoints()[arm.getJoints().length - 1];
		Mat34 referencePose = arm.getReferencePose();

		Vector3D oldRefPos = BhamConverter.convGolemToVec(referencePose.p);
		Matrix oldRefRot = BhamConverter.convGolemToMatrix(referencePose.R);

		BoxShapeDesc pFingerRodShapeDesc = new BoxShapeDescI();
		pFingerRodShapeDesc.dimensions.v1 = FINGERDIAM / 2.0;
		pFingerRodShapeDesc.dimensions.v2 = FINGERLENGTH / 2.0;
		pFingerRodShapeDesc.dimensions.v3 = FINGERDIAM / 2.0;
		pFingerRodShapeDesc.localPose = new Mat34(BhamConverter
				.convMatrixToGolem(oldRefRot), BhamConverter
				.convVecToGolem(oldRefPos));
		pFingerRodShapeDesc.localPose.p.v2 += (FINGERLENGTH / 2.0) - 0.05;
		pFingerRodShapeDesc.localPose.p.v1 -= 0.05;
		pFingerRodShapeDesc.localPose.R = BhamConverter
				.convMatrixToGolem(MathOperation.getRotationAroundZ(10));
		try {
			pEffector.createShape(pFingerRodShapeDesc);
		} catch (Exception e) {
			logger.error(e);
		}

		BoxShapeDesc pFingerRodShapeDesc2 = new BoxShapeDescI();
		pFingerRodShapeDesc2.dimensions.v1 = FINGERDIAM / 2.0;
		pFingerRodShapeDesc2.dimensions.v2 = FINGERLENGTH / 2.0;
		pFingerRodShapeDesc2.dimensions.v3 = FINGERDIAM / 2.0;
		pFingerRodShapeDesc2.localPose = new Mat34(BhamConverter
				.convMatrixToGolem(oldRefRot), BhamConverter
				.convVecToGolem(oldRefPos));
		;
		pFingerRodShapeDesc2.localPose.p.v2 += (FINGERLENGTH / 2.0) - 0.05;
		pFingerRodShapeDesc2.localPose.p.v1 += 0.05;
		pFingerRodShapeDesc2.localPose.R = BhamConverter
				.convMatrixToGolem(MathOperation.getRotationAroundZ(-10));
		try {
			pEffector.createShape(pFingerRodShapeDesc2);
		} catch (Exception e) {
			logger.error(e);
		}

		Mat34 newRefPos = new Mat34(BhamConverter.convMatrixToGolem(oldRefRot),
				BhamConverter.convVecToGolem(new Vector3D(oldRefPos.getX(),
						oldRefPos.getY() + GRIPPERLENGTH, oldRefPos.getZ())));

		arm.setReferencePose(newRefPos);

		// referencePose.p.v2 -= (fingerLength / 2.0);
		//
		// SphereShapeDesc pFingerTipShapeDesc = new SphereShapeDescI();
		// pFingerTipShapeDesc.radius = fingerTipRadius;
		// pFingerTipShapeDesc.localPose = referencePose;
		// pFingerTipShapeDesc.localPose.p.v2 += (fingerLength / 2.0);
		// try {
		// pEffector.createShape(pFingerTipShapeDesc);
		// } catch (Exception e) {
		// logger.error(e);
		// }

		return arm;
	}

	private Vector<ViewPoint> serachInVirtualSceneForGraspingBasePosition(
			BasePositionData currentPosition, Item item) throws ItemException,
			InternalMemoryException {

		Vector3D itemPositionInit = ((Vector3D) manipulator.getItemMemory()
				.getFirstGraspItem().getAttribute(PropertyName.WORLD_POSITION));

		Vector3D itemPosition = new Vector3D(itemPositionInit.getX(),
				itemPositionInit.getY(), itemPositionInit.getZ());

		// TODO nicht doppelt machen
		Matrix rotation1 = MathOperation.getRotationAroundX(MathOperation
				.getRadiant(0));

		Matrix rotation2 = MathOperation.getRotationAroundY(MathOperation
				.getRadiant(0));

		Matrix rotation3 = MathOperation.getRotationAroundZ(MathOperation
				.getRadiant(-90));

		Matrix greifRotation = MathOperation.getMatrixMatrixMultiplication(
				MathOperation.getMatrixMatrixMultiplication(rotation1,
						rotation2), rotation3);

		Vector2D direction = MathOperation.getDirection(currentPosition
				.getPoint(), itemPosition.forgetThirdDimension());

		double currentAngle = currentPosition.getAngle();
		Vector2D currentPos = currentPosition.getPoint();

		BasePositionData virtualPosition = currentPosition;

		Vector<ViewPoint> allPoints = new Vector<ViewPoint>();

		// TODO Konstanten einführen
		double stepsize = 0.05;
		double runner = 0;
		int i = 0;
		double successrate = 0;

		boolean stop = false;

		while (i < 20 && !stop) {
			virtualPosition = new BasePositionData(new Vector2D(currentPos
					.getX()
					+ runner * direction.getX(), currentPos.getY() + runner
					* direction.getY()), currentAngle);

			moveRobotInVirtualScene(virtualPosition);

			ArmError errorVec;
			double tmpError;
			try {
				errorVec = manipulator.getArmConnector().getPosError(
						itemPosition, greifRotation);
				tmpError = MathOperation.getEuclDistance(errorVec
						.getPoseError());
			} catch (ManipulatorException e) {
				tmpError = MathOperation.getEuclDistance(new Vector3D(
						Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
			}

			logger.error(tmpError);

			runner += stepsize;

			if (tmpError < 0.0001) {
				successrate++;
				allPoints.add(new ViewPoint(tmpError, virtualPosition));
				logger.error("Successrate " + successrate);
				if (successrate == 5) {
					stop = true;
				}
			}

			i++;

		}

		// while (MathOperation.getDistance(virtualPosition.getPoint(),
		// itemPosition.forgetThirdDimension()) > 0.1) {
		// logger.error("search");
		// virtualPosition = new BasePositionData(new Vector2D(currentPos
		// .getX()
		// + runner * direction.getX(), currentPos.getY() + runner
		// * direction.getY()), currentAngle);
		//
		// moveRobotInVirtualScene(virtualPosition);
		//
		// ArmError errorVec;
		// double tmpError;
		// try {
		// errorVec = manipulator.getArmConnector().getPosError(
		// itemPosition, goalRot);
		// tmpError = MathOperation.getEuclDistance(errorVec
		// .getPoseError());
		// logger.error("ANGLE: " + errorVec.getAngleError());
		// logger.error("POS: " + tmpError);
		// } catch (ManipulatorException e) {
		// tmpError = MathOperation.getEuclDistance(new Vector3D(
		// Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
		// }
		//
		// if (Math.abs(tmpError - error) < 0.0001) {
		// stop = true;
		// }
		//
		// error = tmpError;
		// allPoints.add(new ViewPoint(error, virtualPosition));
		// runner += stepsize;
		// }

		return allPoints;
	}

	/**
	 * starts a thread to update the position of the robot in the virtual scene
	 */
	public void startPosThread() {
		if (!posThread.isAlive()) {
			posThread = new Thread(
					new UpdatePositionRunnable(this, manipulator));
			posThread.start();
		}
	}

	/**
	 * stops updating the position of the robot in the virtual scene
	 */
	public void stopPosThread() {
		if (posThread.isAlive()) {

			posThread.interrupt();
		}
	}

	/**
	 * starts a thread to update the positions of the item in the virtual scene
	 */
	public void startItemThread() {
		if (!itemThread.isAlive()) {
			itemThread = new Thread(new UpdateItemPositionRunnable(this,
					manipulator));
			itemThread.start();
		}
	}

	/**
	 * stops updating the positions of the items in the virtual scene
	 */
	public void stopItemThread() {
		if (itemThread.isAlive()) {
			itemThread.interrupt();
		}
	}

	/**
	 * update the arm in the virtual scene
	 */
	public void updateArm() {
		Matrix globalRobotPoseRot = BhamConverter.convGolemToMatrix(robot
				.getGlobalPose().R);

		Vec3 globalRobotPoseTrans = robot.getGlobalPose().p;

		Vector3D relativeCoordinate = new Vector3D(0, 0, BASEHIGHT / 2);

		Vector3D newRelCoordinate = MathOperation
				.getMatrixVectorMultiplication(globalRobotPoseRot,
						relativeCoordinate);

		Mat34 globalArmPose = arm.getGlobalPose();

		globalArmPose.p = new Vec3(globalRobotPoseTrans.v1
				+ newRelCoordinate.getX(), globalRobotPoseTrans.v2
				+ newRelCoordinate.getY(), globalRobotPoseTrans.v3
				+ newRelCoordinate.getZ());

		globalArmPose.R = BhamConverter.convMatrixToGolem(MathOperation
				.getMatrixMatrixMultiplication(BhamConverter
						.convGolemToMatrix(robot.getGlobalPose().R),
						initArmRotation));

		arm.setGlobalPose(globalArmPose);
	}

	/**
	 * update the robot position in the virtual scene
	 * 
	 * @param position
	 *            new position value
	 * @return new robot representation
	 * @throws Exception
	 */
	public RigidBodyPrx updateRobot(BasePositionData position) throws Exception {
		RigidBodyDesc robot = new RigidBodyDescI();
		robot.shapes = new ShapeDesc[2];

		BoxShapeDesc base = new BoxShapeDescI();
		base.dimensions.v1 = BASELENGTH / 2;
		base.dimensions.v2 = BASEWIDTH / 2;
		base.dimensions.v3 = BASEHIGHT / 2;

		base.localPose.p = new Vec3(-BASELENGTH / 4, 0, 0);

		robot.shapes[0] = base;

		BoxShapeDesc stick = new BoxShapeDescI();
		stick.dimensions.v1 = STICKWIDTH;
		stick.dimensions.v2 = STICKLENGTH;
		stick.dimensions.v3 = STICKHIGHT / 2;
		stick.localPose.p = new Vec3(-BASELENGTH / 3, 0, BASEHIGHT / 2
				+ STICKHIGHT / 2);
		robot.shapes[1] = stick;

		robot.globalPose.p.v1 = position.getPoint().getX();
		robot.globalPose.p.v2 = position.getPoint().getY();
		robot.globalPose.p.v3 = BASEHIGHT / 2;

		robot.globalPose.R = BhamConverter.convMatrixToGolem(MathOperation
				.getRotationAroundZ(position.getAngle()));

		return RigidBodyPrxHelper.checkedCast(tinyInterface.createActor(robot));
	}

	/**
	 * updates an item position in the virtual scene
	 * 
	 * @param item
	 *            item to change its the position
	 * @return new item representation
	 * @throws Exception
	 */
	public RigidBodyPrx updateObstacle(Item item) throws Exception {
		RigidBodyDesc obstacle = new RigidBodyDescI();
		obstacle.shapes = new ShapeDesc[1];

		// TODO auch was anderes ausser kasten
		BoxShapeDesc obstacleShape = new BoxShapeDescI();
		// TODO itemgroesse
		// TODO itemrotation
		obstacleShape.dimensions.v1 = 0.01;
		obstacleShape.dimensions.v2 = 0.01;
		obstacleShape.dimensions.v3 = 0.01;

		obstacle.shapes[0] = obstacleShape;
		// TODO worldcoordinates to robot

		Vector3D itemPosition;
		try {
			itemPosition = ((Vector3D) item
					.getAttribute(PropertyName.WORLD_POSITION));
			obstacle.globalPose.p.v1 = itemPosition.getX();
			obstacle.globalPose.p.v2 = itemPosition.getY();
			obstacle.globalPose.p.v3 = itemPosition.getZ();

			obstacle.globalPose.R = BhamConverter
					.convMatrixToGolem((Matrix) item
							.getAttribute(PropertyName.WORLD_ROTATION));

		} catch (ItemException e) {
			logger.error(e);
		}

		obstacle.kinematic = true;

		return RigidBodyPrxHelper.checkedCast(tinyInterface
				.createActor(obstacle));
	}

	public RigidBodyPrx generateTable(Item item) throws Exception {
		RigidBodyDesc obstacle = new RigidBodyDescI();
		obstacle.shapes = new ShapeDesc[1];

		// TODO auch was anderes ausser kasten
		BoxShapeDesc obstacleShape = new BoxShapeDescI();
		// TODO itemgroesse
		// TODO itemrotation
		obstacleShape.dimensions.v1 = 0.1;
		obstacleShape.dimensions.v2 = 0.1;
		obstacleShape.dimensions.v3 = (((((Vector3D) item
				.getAttribute(PropertyName.WORLD_POSITION)).getZ())) / 2) - 0.02;
		obstacle.shapes[0] = obstacleShape;
		// TODO worldcoordinates to robot

		Vector3D itemPosition;
		try {
			itemPosition = ((Vector3D) item
					.getAttribute(PropertyName.WORLD_POSITION));
			obstacle.globalPose.p.v1 = itemPosition.getX();
			obstacle.globalPose.p.v2 = itemPosition.getY();
			obstacle.globalPose.p.v3 = (((((Vector3D) item
					.getAttribute(PropertyName.WORLD_POSITION)).getZ())) / 2) - 0.02;

			obstacle.globalPose.R = BhamConverter
					.convMatrixToGolem(MathOperation.getRotationAroundZ(0));

		} catch (ItemException e) {
			logger.error(e);
		}

		obstacle.kinematic = true;

		return RigidBodyPrxHelper.checkedCast(tinyInterface
				.createActor(obstacle));
	}

	/**
	 * gets the communication interface of GOLEM
	 * 
	 * @return communication interface of GOLEM
	 */
	public TinyPrx getTinyInterface() {
		return tinyInterface;
	}

	/**
	 * sets the communication interface of GOLEM
	 * 
	 * @param tinyInterface
	 *            new communication interface of GOLEM
	 */
	public void setTinyInterface(TinyPrx tinyInterface) {
		this.tinyInterface = tinyInterface;
	}

	/**
	 * @return the points
	 */
	public Vector<ViewPoint> getPoints() {
		return points;
	}

	/**
	 * gets the robot representation of the virtual scene
	 * 
	 * @return the robot representation of the virtual scene
	 */
	public RigidBodyPrx getRobot() {
		return robot;
	}

	/**
	 * sets the robot representation
	 * 
	 * @param robot
	 *            new robot representation
	 */
	public void setRobot(RigidBodyPrx robot) {
		this.robot = robot;
	}

	/**
	 * gets all obstacle / item representations
	 * 
	 * @return all obstacle / item representations
	 */
	public Vector<RigidBodyPrx> getObstacles() {
		return obstacles;
	}

	/**
	 * sets all obstacle / item representations
	 * 
	 * @param obstacles
	 *            new obstacle / item representations
	 */
	public void setObstacles(Vector<RigidBodyPrx> obstacles) {
		this.obstacles = obstacles;
	}

	/**
	 * @return the tables
	 */
	public Vector<RigidBodyPrx> getTables() {
		return tables;
	}

	/**
	 * @param tables
	 *            the tables to set
	 */
	public void setTables(Vector<RigidBodyPrx> tables) {
		this.tables = tables;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getInitArmRotation() {
		return initArmRotation;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double getTime() {
		return tinyInterface.getTime();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmPrx getArm() {
		return arm;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void moveRobotInVirtualScene(BasePositionData basePosInVirtualScene) {
		if (getRobot() != null)
			try {
				getTinyInterface().releaseActor(getRobot());
			} catch (Exception e) {
				logger.info(e);
			}
		try {
			setRobot(updateRobot(basePosInVirtualScene));
		} catch (Exception e) {
			logger.error(e);
		}
		updateArm();

		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean removeGraspingBasePoint(ViewPoint point) {
		logger.error("Grasping Point Size before removing: " + points.size());
		boolean test = points.remove(point);
		logger.error("Grasping Point Size after removing: " + points.size());
		return test;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void updateBestGraspingBasePoints(BasePositionData currentPosition,
			Item item) throws ItemException, InternalMemoryException {

		points = serachInVirtualSceneForGraspingBasePosition(currentPosition,
				item);

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ViewPoint getBestGraspingBasePoint() throws ViewPointException {
		if (points == null) {
			throw new ViewPointException(
					"No GraspingBasePoints generated. Use updateBestGraspingBasePoints fist");
		}

		ViewPoint bestGraspingBasePoint = null;
		double bestError = Double.MAX_VALUE;

		for (ViewPoint point : points) {
			if (point.getError() <= bestError) {
				bestGraspingBasePoint = point;
				bestError = point.getError();
			}
		}

		return bestGraspingBasePoint;
	}

	@Override
	public void clearScene() {
		stopItemThread();

		for (RigidBodyPrx table : getTables()) {
			try {
				getTinyInterface().releaseActor(table);
			} catch (Exception e) {
				logger.error(e);
			}
		}

		getTables().clear();

		for (RigidBodyPrx obstacle : getObstacles()) {
			try {
				getTinyInterface().releaseActor(obstacle);
			} catch (Exception e) {
				logger.error(e);
			}
		}
		getObstacles().clear();

	}

	public void addBox(Matrix rotation) {

		RigidBodyDesc obstacle = new RigidBodyDescI();
		obstacle.shapes = new ShapeDesc[1];

		// TODO auch was anderes ausser kasten
		BoxShapeDesc obstacleShape = new BoxShapeDescI();
		// TODO itemgroesse
		// TODO itemrotation
		obstacleShape.dimensions.v1 = 0.2;
		obstacleShape.dimensions.v2 = 0.2;
		obstacleShape.dimensions.v3 = 0.2;

		obstacle.shapes[0] = obstacleShape;
		// TODO worldcoordinates to robot

		obstacle.globalPose.p.v1 = 0;
		obstacle.globalPose.p.v2 = 0;
		obstacle.globalPose.p.v3 = 1;

		obstacle.globalPose.R = BhamConverter.convMatrixToGolem(rotation);

		obstacle.kinematic = true;

		try {
			RigidBodyPrxHelper.checkedCast(tinyInterface.createActor(obstacle));
		} catch (ExTiny e) {
			logger.error(e);
		}
	}
}
