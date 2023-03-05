package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.*;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {
	private static final double MAX_VOLTAGE = 12.0; // volts
	public static final double VELOCITY_RANGE = 2.5; // 1.5 m/s
	private static final double MAX_VELOCITY = 4.5; // m/s
	public static final double ANGULAR_VELOCITY_RANGE = Math.PI / 2; // rad/s

	private static final double DT_TRACKWIDTH = 0.36195; // m
	private static final double DT_WHEELBASE = 0.76835; // m

	private static final Translation2d[] MODULE_TRANSLATIONS = {
		// Front left
		new Translation2d(DT_WHEELBASE / 2.0, DT_TRACKWIDTH / 2.0),
		// Front right
		new Translation2d(DT_WHEELBASE / 2.0, -DT_TRACKWIDTH / 2.0),
		// Back left
		new Translation2d(-DT_WHEELBASE / 2.0, DT_TRACKWIDTH / 2.0),
		// Back right
		new Translation2d(-DT_WHEELBASE / 2.0, -DT_TRACKWIDTH / 2.0)
	};

	private final SwerveDriveKinematics m_kinematics =
			new SwerveDriveKinematics(MODULE_TRANSLATIONS);

	private static final int GYRO_ID = 0;

	private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 13;
	private static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 12;
	private static final int FRONT_LEFT_MODULE_ENCODER_ID = 12;

	private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 11;
	private static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 10;
	private static final int FRONT_RIGHT_MODULE_ENCODER_ID = 10;

	private static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 17;
	private static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 16;
	private static final int BACK_LEFT_MODULE_ENCODER_ID = 16;

	private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 15;
	private static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 14;
	private static final int BACK_RIGHT_MODULE_ENCODER_ID = 14;

	private static final double FRONT_LEFT_MODULE_STEER_OFFSET =
			-Math.toRadians(99.49 - 180.0); // rads 288.8
	private static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(23.29); // rads
	private static final double BACK_LEFT_MODULE_STEER_OFFSET =
			-Math.toRadians(192.04); // 190.45 rads
	private static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(329.67); // rads

	private final SwerveDrive swerveDrive;

	private Rotation2d m_simYaw;

	private final Field2dWrapper m_field;

	private final Pose2d[] m_swerveModulePoses = {
		new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()
	};

	private PathPlannerTrajectory m_currentTrajectory;
	private boolean m_isRunningTrajectory;

	public Swerve() {

		try {
			swerveDrive = new SwerveParser(Constants.SWERVE_DIRECTORY).createSwerveDrive();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}

		m_simYaw = new Rotation2d();

		m_field = new Field2dWrapper();

		SmartDashboard.putData("Field", m_field);

		//		m_field.getObject("Control Points").setPoses(ObstacleAvoidance.getControlPoints());
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	public Rotation2d getHeading() {
		return swerveDrive.getYaw();
	}

	public void lockPose() {
		swerveDrive.lockPose();
	}

	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	public void drive(
			Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
	}

	public void stop() {
		drive(new Translation2d(0.0, 0.0), 0.0, false, false);
	}

	@Override
	public void periodic() {

		swerveDrive.updateOdometry();

		// TODO: Add StdDevs if needed
		AprilTagTracker.getEstimatedGlobalPose(
						swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition())
				.ifPresentOrElse(
						estimatedRobotPose -> {
							swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
									estimatedRobotPose.estimatedPose.toPose2d(),
									estimatedRobotPose.timestampSeconds);
							m_field.getObject("Cam Est Pos")
									.setPose(estimatedRobotPose.estimatedPose.toPose2d());
						},
						() -> {
							//							System.out.println("NO APRIL TAGS");
						});

		m_field.setRobotPose(swerveDrive.getPose());

		//		SwerveModule[] modules = {
		//			m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule
		//		};
		//
		//		for (int i = 0; i < 4; i++) {
		//			Translation2d updatedPositions =
		//					MODULE_TRANSLATIONS[i]
		//							.rotateBy(getPose().getRotation())
		//							.plus(getPose().getTranslation());
		//
		//			m_swerveModulePoses[i] =
		//					new Pose2d(
		//							updatedPositions,
		//							new Rotation2d(modules[i].getSteerAngle()).plus(getGyroHeading()));
		//		}

		m_field.getObject("aSwerve Modules")
				.setPoses(swerveDrive.getSwerveModulePoses(swerveDrive.getPose()));
	}

	public void simulationPeriodic() {}

	public ChassisSpeeds getTargetSpeeds(
			double xInput, double yInput, double headingX, double headingY) {
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return swerveDrive.swerveController.getTargetSpeeds(
				xInput, yInput, headingX, headingY, getHeading().getRadians());
	}

	public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
		xInput = Math.pow(xInput, 3);
		yInput = Math.pow(yInput, 3);
		return swerveDrive.swerveController.getTargetSpeeds(
				xInput, yInput, angle.getRadians(), getHeading().getRadians());
	}

	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	public Field2d getField() {
		return m_field;
	}

	public void setRunningTrajectory(boolean isRunningTrajectory) {
		m_isRunningTrajectory = isRunningTrajectory;
	}

	public PathPlannerTrajectory getTrajectory() {
		return m_currentTrajectory;
	}

	public void setTrajectory(PathPlannerTrajectory trajectory) {
		m_currentTrajectory = trajectory;
	}
}
