package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.swervedrivespecialties.swervelib.GearRatios.GearRatio;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.*;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {
	private static final double MAX_VOLTAGE = 12.0; // volts
	public static final double MAX_VELOCITY = 1.5; // m/s
	public static final double MAX_ANGULAR_VELOCITY = 1.5; // m/s

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

	private static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(288.8); // rads
	private static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(26.63); // rads
	private static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(190.45); // rads
	private static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(326.51); // rads

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private final WPI_Pigeon2 m_gyro;

	private Rotation2d m_simYaw;

	private ChassisSpeeds m_chassisSpeeds;

	private final HeadingController m_headingController =
			new HeadingController(
					0.005, // Stabilization kP
					0.0, // Stabilization kD
					1.75, // Lock kP
					0.0, // Lock kI
					0.0, // Lock kD
					2.0, // Turn in place kP
					0.0, // Turn in place kI
					0.0 // Turn in place kD
					);

	private final SwerveDrivePoseEstimator m_poseEstimator;
	private final Field2dWrapper m_field;

	private final Pose2d[] m_swerveModulePoses = {
		new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()
	};

	private PathPlannerTrajectory m_currentTrajectory;
	private boolean m_isRunningTrajectory;

	public Swerve() {
		ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

		m_frontLeftModule =
				new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
						.withGearRatio(GearRatio.L3.getConfiguration())
						.withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR_ID)
						.withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR_ID)
						.withLayout(
								tab.getLayout("Front Left Module", BuiltInLayouts.kList)
										.withSize(2, 4)
										.withPosition(0, 0))
						.withSteerEncoderPort(FRONT_LEFT_MODULE_ENCODER_ID)
						.withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
						.build();

		m_frontRightModule =
				new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
						.withGearRatio(GearRatio.L3.getConfiguration())
						.withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID)
						.withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR_ID)
						.withLayout(
								tab.getLayout("Front Right Module", BuiltInLayouts.kList)
										.withSize(2, 4)
										.withPosition(2, 0))
						.withSteerEncoderPort(FRONT_RIGHT_MODULE_ENCODER_ID)
						.withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
						.build();

		m_backLeftModule =
				new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
						.withGearRatio(GearRatio.L3.getConfiguration())
						.withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR_ID)
						.withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR_ID)
						.withLayout(
								tab.getLayout("Back Left Module", BuiltInLayouts.kList)
										.withSize(2, 4)
										.withPosition(4, 0))
						.withSteerEncoderPort(BACK_LEFT_MODULE_ENCODER_ID)
						.withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
						.build();

		m_backRightModule =
				new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerFalcon500())
						.withGearRatio(GearRatio.L3.getConfiguration())
						.withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR_ID)
						.withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR_ID)
						.withLayout(
								tab.getLayout("Back Right Module", BuiltInLayouts.kList)
										.withSize(2, 4)
										.withPosition(6, 0))
						.withSteerEncoderPort(BACK_RIGHT_MODULE_ENCODER_ID)
						.withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
						.build();

		m_gyro = new WPI_Pigeon2(GYRO_ID);

		m_simYaw = new Rotation2d();

		m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		m_poseEstimator =
				new SwerveDrivePoseEstimator(
						m_kinematics, getGyroHeading(), getModulePositions(), new Pose2d());

		m_field = new Field2dWrapper();

		SmartDashboard.putData("Field", m_field);

		m_field.getObject("Control Points").setPoses(ObstacleAvoidance.getControlPoints());
	}

	public void zeroGyro() {
		m_gyro.setYaw(0.0);
	}

	public Rotation2d getGyroHeading() {
		return Rotation2d.fromDegrees(m_gyro.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
		m_headingController.stabiliseHeading();
	}

	public void stop() {
		drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}

	private SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
			m_frontLeftModule.getPosition(),
			m_frontRightModule.getPosition(),
			m_backLeftModule.getPosition(),
			m_backRightModule.getPosition()
		};
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

		SmartDashboard.putNumber("Gyro", getGyroHeading().getDegrees());

		SmartDashboard.putNumber("Pose X", m_poseEstimator.getEstimatedPosition().getX());
		SmartDashboard.putNumber("Pose Y", m_poseEstimator.getEstimatedPosition().getY());

		if (!m_isRunningTrajectory) {
			m_headingController.update(m_chassisSpeeds, getGyroHeading());
		}

		m_frontLeftModule.set(
				states[0].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(
				states[1].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[1].angle.getRadians());
		m_backLeftModule.set(
				states[2].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[2].angle.getRadians());
		m_backRightModule.set(
				states[3].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[3].angle.getRadians());

		m_poseEstimator.update(getGyroHeading(), getModulePositions());

		m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

		// TODO: Add StdDevs if needed
		AprilTagTracker.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition())
				.ifPresent(
						estimatedRobotPose ->
								m_poseEstimator.addVisionMeasurement(
										estimatedRobotPose.estimatedPose.toPose2d(),
										estimatedRobotPose.timestampSeconds));

		SwerveModule[] modules = {
			m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule
		};

		for (int i = 0; i < 4; i++) {
			Translation2d updatedPositions =
					MODULE_TRANSLATIONS[i]
							.rotateBy(getPose().getRotation())
							.plus(getPose().getTranslation());

			m_swerveModulePoses[i] =
					new Pose2d(
							updatedPositions,
							new Rotation2d(modules[i].getSteerAngle()).plus(getGyroHeading()));
		}

		m_field.getObject("Swerve Modules").setPoses(m_swerveModulePoses);

		m_frontLeftModule.offsetEncoder(FRONT_LEFT_MODULE_STEER_OFFSET);
		m_frontRightModule.offsetEncoder(FRONT_RIGHT_MODULE_STEER_OFFSET);
		m_backLeftModule.offsetEncoder(BACK_LEFT_MODULE_STEER_OFFSET);
		m_backRightModule.offsetEncoder(BACK_RIGHT_MODULE_STEER_OFFSET);
	}

	public void simulationPeriodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

		SmartDashboard.putNumber("Gyro", getGyroHeading().getDegrees());

		SmartDashboard.putNumber("Pose X", m_poseEstimator.getEstimatedPosition().getX());
		SmartDashboard.putNumber("Pose Y", m_poseEstimator.getEstimatedPosition().getY());

		if (!m_isRunningTrajectory) {
			m_headingController.update(m_chassisSpeeds, getGyroHeading());
		}

		m_frontLeftModule.setSim(states[0].speedMetersPerSecond, states[0].angle.getRadians());
		m_frontRightModule.setSim(states[1].speedMetersPerSecond, states[1].angle.getRadians());
		m_backLeftModule.setSim(states[2].speedMetersPerSecond, states[2].angle.getRadians());
		m_backRightModule.setSim(states[3].speedMetersPerSecond, states[3].angle.getRadians());

		m_simYaw =
				m_simYaw.plus(
						new Rotation2d(
								m_chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_TIME));

		m_gyro.setYaw(m_simYaw.getDegrees());

		m_poseEstimator.update(m_simYaw, getModulePositions());

		m_field.setRobotPose(
				new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), m_simYaw));

		m_field.updateSims(getPose());

		m_frontLeftModule.offsetEncoder(FRONT_LEFT_MODULE_STEER_OFFSET);
		m_frontRightModule.offsetEncoder(FRONT_RIGHT_MODULE_STEER_OFFSET);
		m_backLeftModule.offsetEncoder(BACK_LEFT_MODULE_STEER_OFFSET);
		m_backRightModule.offsetEncoder(BACK_RIGHT_MODULE_STEER_OFFSET);
	}

	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	public void setPose(Pose2d pose, Rotation2d heading) {
		Pose2d poseNoRot = new Pose2d(pose.getTranslation(), new Rotation2d());

		m_poseEstimator.resetPosition(heading, getModulePositions(), poseNoRot);
	}

	public ChassisSpeeds getVelocity() {
		return m_chassisSpeeds;
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
