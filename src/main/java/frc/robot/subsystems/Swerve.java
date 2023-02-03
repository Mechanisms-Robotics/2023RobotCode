package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
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
import frc.lib.Mk4iSwerveModuleHelper;
import frc.lib.Mk4iSwerveModuleHelper.GearRatio;
import frc.lib.SwerveModule;

/** The base swerve drive class, controls all swerve modules in coordination. */
public class Swerve extends SubsystemBase {
	private static final double MAX_VOLTAGE = 12.0; // volts
	public static final double MAX_VELOCITY = 4.5; // m/s
	public static final double MAX_ANGULAR_VELOCITY = 1.5; // m/s

	private static final double DT_TRACKWIDTH = 0.36195; // m
	private static final double DT_WHEELBASE = 0.76835; // m

	private final SwerveDriveKinematics m_kinematics =
			new SwerveDriveKinematics(
					// Front left
					new Translation2d(DT_TRACKWIDTH / 2.0, DT_WHEELBASE / 2.0),
					// Front right
					new Translation2d(DT_TRACKWIDTH / 2.0, -DT_WHEELBASE / 2.0),
					// Back left
					new Translation2d(-DT_TRACKWIDTH / 2.0, DT_WHEELBASE / 2.0),
					// Back right
					new Translation2d(-DT_TRACKWIDTH / 2.0, -DT_WHEELBASE / 2.0));

	private static final int GYRO_ID = 0;

	private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 10;
	private static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 11;
	private static final int FRONT_LEFT_MODULE_ENCODER_ID = 10;

	private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 12;
	private static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 13;
	private static final int FRONT_RIGHT_MODULE_ENCODER_ID = 12;

	private static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 14;
	private static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 15;
	private static final int BACK_LEFT_MODULE_ENCODER_ID = 14;

	private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 16;
	private static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 17;
	private static final int BACK_RIGHT_MODULE_ENCODER_ID = 16;

	private static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // rads
	private static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // rads
	private static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // rads
	private static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // rads

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private final PigeonIMU m_gyro;

	private ChassisSpeeds m_chassisSpeeds;

	private final SwerveDrivePoseEstimator m_poseEstimator;
	private final Field2d m_field;

	public Swerve() {
		ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

		m_frontLeftModule =
				Mk4iSwerveModuleHelper.createFalcon500(
						tab.getLayout("Front Left Module", BuiltInLayouts.kList)
								.withSize(2, 4)
								.withPosition(0, 0),
						GearRatio.L3,
						FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
						FRONT_LEFT_MODULE_STEER_MOTOR_ID,
						FRONT_LEFT_MODULE_ENCODER_ID,
						FRONT_LEFT_MODULE_STEER_OFFSET);

		m_frontRightModule =
				Mk4iSwerveModuleHelper.createFalcon500(
						tab.getLayout("Front Right Module", BuiltInLayouts.kList)
								.withSize(2, 4)
								.withPosition(2, 0),
						GearRatio.L3,
						FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
						FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
						FRONT_RIGHT_MODULE_ENCODER_ID,
						FRONT_RIGHT_MODULE_STEER_OFFSET);

		m_backLeftModule =
				Mk4iSwerveModuleHelper.createFalcon500(
						tab.getLayout("Back Left Module", BuiltInLayouts.kList)
								.withSize(2, 4)
								.withPosition(4, 0),
						GearRatio.L3,
						BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
						BACK_LEFT_MODULE_STEER_MOTOR_ID,
						BACK_LEFT_MODULE_ENCODER_ID,
						BACK_LEFT_MODULE_STEER_OFFSET);

		m_backRightModule =
				Mk4iSwerveModuleHelper.createFalcon500(
						tab.getLayout("Back Right Module", BuiltInLayouts.kList)
								.withSize(2, 4)
								.withPosition(6, 0),
						GearRatio.L3,
						BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
						BACK_RIGHT_MODULE_STEER_MOTOR_ID,
						BACK_RIGHT_MODULE_ENCODER_ID,
						BACK_RIGHT_MODULE_STEER_OFFSET);

		m_gyro = new PigeonIMU(GYRO_ID);

		m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		m_poseEstimator =
				new SwerveDrivePoseEstimator(
						m_kinematics, getGyroHeading(), getModulePositions(), new Pose2d());

		m_field = new Field2d();

		SmartDashboard.putData("Field", m_field);
	}

	public void zeroGyro() {
		m_gyro.setFusedHeading(0.0);
	}

	public Rotation2d getGyroHeading() {
		return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	private SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
			m_frontLeftModule.getModulePosition(),
			m_frontRightModule.getModulePosition(),
			m_backLeftModule.getModulePosition(),
			m_backRightModule.getModulePosition()
		};
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

		m_frontLeftModule.set(
				states[0].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(
				states[1].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_backLeftModule.set(
				states[2].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_backRightModule.set(
				states[3].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
				states[0].angle.getRadians());

		m_poseEstimator.update(getGyroHeading(), getModulePositions());

		m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
	}
}
