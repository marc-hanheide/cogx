package manipulation.core.cogx.virtualSceneConnector;

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

import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.cogx.virtualSceneConnector.initObjects.ArmDescI;
import manipulation.core.cogx.virtualSceneConnector.initObjects.BoxShapeDescI;
import manipulation.core.cogx.virtualSceneConnector.initObjects.KatanaArmDescI;
import manipulation.core.cogx.virtualSceneConnector.initObjects.PlaneShapeDescI;
import manipulation.core.cogx.virtualSceneConnector.initObjects.RigidBodyDescI;
import manipulation.core.share.Manipulator;
import manipulation.core.share.armConnector.ArmConnector.ArmName;
import manipulation.core.share.exceptions.CalibrationException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.VisionModel;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

/**
 * represents a connector to the GOLEM virtual scene
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXVirtualSceneConnector implements VirtualSceneConnector {
	private Manipulator manipulator;

	private double baseHeight, baseLength, baseWidth, stichHeight, stickLength,
			stickWidth, fingerLength, fingerDiam, gripperLength;

	private Logger logger = Logger.getLogger(this.getClass());

	private Vector<RigidBodyPrx> obstacles;
	public TinyPrx tinyInterface;
	private RigidBodyPrx robot = null;
	private ArmPrx arm;

	private Thread posThread, itemThread;

	/**
	 * constructor of the connector to the GOLEM virtual scene
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public CogXVirtualSceneConnector(Manipulator manipulator) {
		obstacles = new Vector<RigidBodyPrx>();
		this.manipulator = manipulator;

		try {
			baseHeight = (manipulator.getCalibrationConnector()
					.getRobToArmTranslation().getZ());
		} catch (CalibrationException e) {
			logger.error(e);
		}

		baseLength = 0.51;
		baseWidth = 0.38;

		stichHeight = 1;
		stickLength = 0.05;
		stickWidth = 0.05;

		fingerLength = 0.15;
		fingerDiam = 0.005;
		gripperLength = 0.08;

		posThread = new Thread(new UpdatePositionRunnable(this, manipulator));
		itemThread = new Thread(new UpdateItemPositionRunnable(this,
				manipulator));

		Ice.Communicator ic = null;
		ic = Ice.Util.initialize();

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
			arm = createArm(manipulator.getConfiguration().getArmName());
		} catch (ExTiny e) {
			logger.error(e);
		}

		posThread.start();
		itemThread.start();
	}

	/**
	 * create the ground plane in the virtual scene
	 * 
	 * @return ground plane
	 * @throws ExTiny
	 */
	private RigidBodyPrx createGroundPlane() throws ExTiny {
		RigidBodyDesc pGroundPlaneDesc = new RigidBodyDescI();
		PlaneShapeDesc pGroundPlaneShapeDesc = new PlaneShapeDescI();
		pGroundPlaneDesc.shapes[0] = pGroundPlaneShapeDesc;
		return RigidBodyPrxHelper.checkedCast(tinyInterface
				.createActor(pGroundPlaneDesc));
	}

	/**
	 * creates the manipulator in the virtual scene
	 * 
	 * @param armName
	 *            name of the arm to create
	 * @return created manipulator
	 */
	private ArmPrx createArm(ArmName armName) {
		Vec3 globalArmPosePos = null;
		try {
			globalArmPosePos = new Vec3(manipulator.getCalibrationConnector()
					.getRobToArmTranslation().getX(), manipulator
					.getCalibrationConnector().getRobToArmTranslation().getY(),
					manipulator.getCalibrationConnector()
							.getRobToArmTranslation().getZ());
		} catch (CalibrationException e) {
			logger.error(e);
		}
		Mat33 globalArmPoseRot = null;
		try {
			globalArmPoseRot = CogXConverter.convMatrixToGolem(manipulator
					.getCalibrationConnector().getRobToArmRotation());
		} catch (CalibrationException e) {
			logger.error(e);
		}
		ArmPrx arm = null;
		logger.debug("Creating Arm...");

		if (armName == ArmName.SIMULATION) {
			ArmDesc pArmDesc = new ArmDescI();
			pArmDesc.path = "GolemDeviceKatana300Sim";
			pArmDesc.globalPose.p = globalArmPosePos;
			pArmDesc.globalPose.R = globalArmPoseRot;

			try {
				arm = ArmPrxHelper.checkedCast(tinyInterface
						.createActor(pArmDesc));
			} catch (ExTiny e) {
				logger.error(e);
			}

		} else if (armName == ArmName.KATANA300) {
			KatanaArmDesc pArmDesc = new KatanaArmDescI();
			pArmDesc.path = "GolemDeviceKatana300";
			pArmDesc.bGripper = true;
			pArmDesc.globalPose.p = globalArmPosePos;
			pArmDesc.globalPose.R = globalArmPoseRot;

			try {
				arm = KatanaArmPrxHelper.checkedCast(tinyInterface
						.createActor(pArmDesc));
			} catch (ExTiny e) {
				logger.error(e);
			}

		} else if (armName == ArmName.KATANA450) {
			KatanaArmDesc pArmDesc = new KatanaArmDescI();
			pArmDesc.path = "GolemDeviceKatana450";
			pArmDesc.bGripper = true;
			pArmDesc.globalPose.p = globalArmPosePos;
			pArmDesc.globalPose.R = globalArmPoseRot;

			try {
				arm = KatanaArmPrxHelper.checkedCast(tinyInterface
						.createActor(pArmDesc));
			} catch (ExTiny e) {
				logger.error(e);
			}
		} else {
			logger.error("Cannot create arm");
		}

		JointPrx pEffector = arm.getJoints()[arm.getJoints().length - 1];
		Mat34 referencePose = arm.getReferencePose();

		Vector3D oldRefPos = CogXConverter.convGolemToVec(referencePose.p);
		Matrix oldRefRot = CogXConverter.convGolemToMatrix(referencePose.R);

		BoxShapeDesc pFingerRodShapeDesc = new BoxShapeDescI();
		pFingerRodShapeDesc.dimensions.v1 = fingerDiam / 2.0;
		pFingerRodShapeDesc.dimensions.v2 = fingerLength / 2.0;
		pFingerRodShapeDesc.dimensions.v3 = fingerDiam / 2.0;
		pFingerRodShapeDesc.localPose = new Mat34(
				CogXConverter.convMatrixToGolem(oldRefRot),
				CogXConverter.convVecToGolem(oldRefPos));
		pFingerRodShapeDesc.localPose.p.v2 += (fingerLength / 2.0) - 0.05;
		pFingerRodShapeDesc.localPose.p.v1 -= 0.05;
		pFingerRodShapeDesc.localPose.R = CogXConverter
				.convMatrixToGolem(MathOperation.getRotationAroundZ(10));

		try {
			pEffector.createShape(pFingerRodShapeDesc);
		} catch (ExTiny e) {
			logger.error(e);
		}

		BoxShapeDesc pFingerRodShapeDesc2 = new BoxShapeDescI();
		pFingerRodShapeDesc2.dimensions.v1 = fingerDiam / 2.0;
		pFingerRodShapeDesc2.dimensions.v2 = fingerLength / 2.0;
		pFingerRodShapeDesc2.dimensions.v3 = fingerDiam / 2.0;
		pFingerRodShapeDesc2.localPose = new Mat34(
				CogXConverter.convMatrixToGolem(oldRefRot),
				CogXConverter.convVecToGolem(oldRefPos));
		pFingerRodShapeDesc2.localPose.p.v2 += (fingerLength / 2.0) - 0.05;
		pFingerRodShapeDesc2.localPose.p.v1 += 0.05;
		pFingerRodShapeDesc2.localPose.R = CogXConverter
				.convMatrixToGolem(MathOperation.getRotationAroundZ(-10));

		try {
			pEffector.createShape(pFingerRodShapeDesc2);
		} catch (ExTiny e) {
			logger.error(e);
		}

		Mat34 newRefPos = new Mat34(CogXConverter.convMatrixToGolem(oldRefRot),
				CogXConverter.convVecToGolem(new Vector3D(oldRefPos.getX(),
						oldRefPos.getY() + gripperLength, oldRefPos.getZ())));

		arm.setReferencePose(newRefPos);

		return arm;
	}

	/**
	 * updates the position of the arm (while moving the robot)
	 */
	public void updateArm() {
		Matrix globalRobotPoseRot = CogXConverter.convGolemToMatrix(robot
				.getGlobalPose().R);

		Vec3 globalRobotPoseTrans = robot.getGlobalPose().p;

		Vector3D relativeCoordinate = null;
		try {
			relativeCoordinate = new Vector3D(
					manipulator.getCalibrationConnector()
							.getRobToArmTranslation().getX() / 2, manipulator
							.getCalibrationConnector().getRobToArmTranslation()
							.getY() / 2, baseHeight / 2);
		} catch (CalibrationException e) {
			logger.error(e);
		}

		Vector3D newRelCoordinate = MathOperation
				.getMatrixVectorMultiplication(globalRobotPoseRot,
						relativeCoordinate);

		try {
			Mat34 globalArmPose = arm.getGlobalPose();

			globalArmPose.p = new Vec3(globalRobotPoseTrans.v1
					+ newRelCoordinate.getX(), globalRobotPoseTrans.v2
					+ newRelCoordinate.getY(), globalRobotPoseTrans.v3
					+ newRelCoordinate.getZ());

			globalArmPose.R = CogXConverter.convMatrixToGolem(MathOperation
					.getMatrixMatrixMultiplication(CogXConverter
							.convGolemToMatrix(robot.getGlobalPose().R),
							manipulator.getCalibrationConnector()
									.getRobToArmRotation()));

			arm.setGlobalPose(globalArmPose);
		} catch (CalibrationException e) {
			logger.error(e);
		}

	}

	/**
	 * updates the position of the robot in the virtual scene to the given base
	 * position
	 * 
	 * @param position
	 *            current position of the robot
	 * @return moved robot
	 * @throws ExTiny
	 */
	public RigidBodyPrx updateRobot(BasePositionData position) throws ExTiny {
		RigidBodyDesc robot = new RigidBodyDescI();
    // if the table obstacle hack is enabled, we add an extra part to the
    // robot shape
    if(manipulator.getConfiguration().isEnabledTableObstacleHack())
      robot.shapes = new ShapeDesc[3];
    else
      robot.shapes = new ShapeDesc[2];

		BoxShapeDesc base = new BoxShapeDescI();
		base.dimensions.v1 = baseLength / 2;
		base.dimensions.v2 = baseWidth / 2;
		base.dimensions.v3 = baseHeight / 2;
		base.localPose.p = new Vec3(-baseLength / 4, 0, 0);
		robot.shapes[0] = base;

		BoxShapeDesc stick = new BoxShapeDescI();
		stick.dimensions.v1 = stickWidth;
		stick.dimensions.v2 = stickLength;
		stick.dimensions.v3 = stichHeight / 2;
		stick.localPose.p = new Vec3(-baseLength / 3, 0, baseHeight / 2
				+ stichHeight / 2);
		robot.shapes[1] = stick;

    if(manipulator.getConfiguration().isEnabledTableObstacleHack()) {
      // safety obstacle for tables
      BoxShapeDesc safety = new BoxShapeDescI();
      safety.dimensions.v1 = 1;
      safety.dimensions.v2 = 1;
      safety.dimensions.v3 = 0.25;
      safety.localPose.p = new Vec3(1.3, 0, -baseHeight/2 + 0.25);
      robot.shapes[2] = safety;
    }

		robot.globalPose.p.v1 = position.getPoint().getX();
		robot.globalPose.p.v2 = position.getPoint().getY();
		robot.globalPose.p.v3 = baseHeight / 2;

		robot.globalPose.R = CogXConverter.convMatrixToGolem(MathOperation
				.getRotationAroundZ(position.getAngle()));

		return RigidBodyPrxHelper.checkedCast(tinyInterface.createActor(robot));
	}

	/**
	 * updates the position of a given item in the virtual scene
	 * 
	 * @param item
	 *            obstacle to update in the virtual scene
	 * @return obstacle updated obstacle
	 * @throws ItemException
	 * @throws ExTiny
	 */
	public RigidBodyPrx updateObstacle(Item item) throws ItemException, ExTiny {
		RigidBodyDesc obstacle = new RigidBodyDescI();
		obstacle.shapes = new ShapeDesc[1];

		BoxShapeDesc obstacleShape = new BoxShapeDescI();

		VisionModel visionModel = (VisionModel) item
				.getAttribute(PropertyName.MODEL);

		obstacleShape.dimensions.v1 = (visionModel.getHighestXValue() - visionModel
				.getLowestXValue()) / 2;
		obstacleShape.dimensions.v2 = (visionModel.getHighestYValue() - visionModel
				.getLowestYValue()) / 2;
		obstacleShape.dimensions.v3 = (visionModel.getHighestZValue() - visionModel
				.getLowestZValue()) / 2;

		obstacle.shapes[0] = obstacleShape;

		Vector3D itemPosition;
		try {
			itemPosition = ((Vector3D) item
					.getAttribute(PropertyName.WORLD_POSITION));
			obstacle.globalPose.p.v1 = itemPosition.getX();
			obstacle.globalPose.p.v2 = itemPosition.getY();
			obstacle.globalPose.p.v3 = itemPosition.getZ();

			obstacle.globalPose.R = CogXConverter
					.convMatrixToGolem((Matrix) item
							.getAttribute(PropertyName.WORLD_ROTATION));
		} catch (ItemException e) {
			logger.error(e);
		}

		obstacle.kinematic = true;

		return RigidBodyPrxHelper.checkedCast(tinyInterface
				.createActor(obstacle));
	}

	/**
	 * gets the tiny ice interface
	 * 
	 * @return the tiny interface
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
	 * gets the robot
	 * 
	 * @return robot of the virtual scene
	 */
	public RigidBodyPrx getRobot() {
		return robot;
	}

	/**
	 * sets the robot in the virtual scene
	 * 
	 * @param robot
	 *            robot to set
	 */
	public void setRobot(RigidBodyPrx robot) {
		this.robot = robot;
	}

	/**
	 * gets all obstacles
	 * 
	 * @return all obstacles
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
	 * gets the virtual arm
	 * 
	 * @return arm
	 */
	public ArmPrx getArm() {
		return arm;
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
	public void clearScene() {
		itemThread.interrupt();
		for (RigidBodyPrx obstacle : getObstacles()) {
			try {
				getTinyInterface().releaseActor(obstacle);
			} catch (ExTiny e) {
				logger.error(e);
			}
		}
		getObstacles().clear();
	}
}
