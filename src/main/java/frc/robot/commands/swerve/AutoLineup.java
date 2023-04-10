package frc.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoLineup extends CommandBase {
	private static final double TARGET_AREA = 0.24;
	private static final double TARGET_YAW = 10.0;
	private static final double TARGET_GYRO = 0.0;

	private static final double ALLOWABLE_AREA_ERROR = 0.02; // 0.02
	private static final double ALLOWABLE_YAW_ERROR = 1.0; // 1.0
	private static final double ALLOWABLE_GYRO_ERROR = 0.075; // 0.075

	private static final double MIN_DISTANCE_VALUE = 0.2; // 0.01
	private static final double MIN_STRAFE_VALUE = 0.01; // 0.175
	private static final double MIN_ROTATION_VALUE = 0.1875; // 0.1875

	private static final double AT_POSITION_TIME = 0.01; // seconds

	private final Swerve m_swerve;
	private final Limelight m_limelight;

	private final ProfiledPIDController m_strafePIDController;
	private final ProfiledPIDController m_distancePIDController;
	private final ProfiledPIDController m_rotationPIDController;

	private final Timer m_atPositionTimer = new Timer();

	private double areaError = 1000.0;
	private double yawError = 1000.0;
	private double gyroError = 1000.0;

	private boolean m_atPosition = false;

	public AutoLineup(Swerve swerve, Limelight limelight) {
		m_swerve = swerve;
		m_limelight = limelight;

		m_strafePIDController =
				new ProfiledPIDController(
						0.05,
						0.0,
						0.0,
						new Constraints(
								2.0, // m/s
								1.0 // m/s
								)); // 0.015

		m_distancePIDController =
				new ProfiledPIDController(
						3.0,
						0.0,
						0.0,
						new Constraints(
								2.0, // m/s
								1.0 // m/s
								)); // 4.0

		m_rotationPIDController =
				new ProfiledPIDController(
						0.0, 0.0, 0.0, new Constraints(0.2 * Math.PI, 0.2 * Math.PI)); // 4.0

		m_strafePIDController.setTolerance(ALLOWABLE_YAW_ERROR);
		m_distancePIDController.setTolerance(ALLOWABLE_AREA_ERROR);
		m_rotationPIDController.setTolerance(ALLOWABLE_GYRO_ERROR);

		addRequirements(swerve, limelight);
	}

	@Override
	public void initialize() {
		m_swerve.setNeutralMode(NeutralMode.Brake);
		m_limelight.setLEDs(true);

		m_atPositionTimer.stop();
		m_atPositionTimer.reset();

		m_atPosition = false;
	}

	@Override
	public void execute() {
		m_limelight.setLEDs(true);

		PhotonTrackedTarget bestTarget = m_limelight.getBestTarget();

		if (bestTarget == null) {
			System.out.println("NO TARGET");

			m_swerve.setNeutralMode(NeutralMode.Brake);
			m_swerve.stop();
			return;
		}

		System.out.println("TARGET");

		areaError = TARGET_AREA - bestTarget.getArea();
		yawError = TARGET_YAW - bestTarget.getYaw();
		gyroError = TARGET_GYRO - m_swerve.getGyroHeading().getRadians();

		boolean atPosition =
				(Math.abs(areaError) <= ALLOWABLE_AREA_ERROR
						&& Math.abs(yawError) <= ALLOWABLE_YAW_ERROR
						&& Math.abs(gyroError) <= ALLOWABLE_GYRO_ERROR);

		if (atPosition && !m_atPositionTimer.hasElapsed(0.01)) {
			m_atPositionTimer.start();
		} else if (!atPosition && m_atPositionTimer.hasElapsed(0.01)) {
			m_atPositionTimer.stop();
			m_atPositionTimer.reset();
		}

		if (m_atPositionTimer.hasElapsed(AT_POSITION_TIME)) {
			m_atPosition = true;
		}

		double vx =
				!(Math.abs(areaError) <= ALLOWABLE_AREA_ERROR)
						? m_distancePIDController.calculate(areaError)
						: 0.0;
		double vy =
				!(Math.abs(yawError) <= ALLOWABLE_YAW_ERROR)
						? m_strafePIDController.calculate(yawError)
						: 0.0;

		if (Math.abs(vx) > 0.0 && Math.abs(vx) < MIN_DISTANCE_VALUE) {
			vx = MIN_DISTANCE_VALUE * Math.signum(vx);
		}

		if (Math.abs(vy) > 0.0 && Math.abs(vy) < MIN_STRAFE_VALUE) {
			vy = MIN_STRAFE_VALUE * Math.signum(vy);
		}

		System.out.println("areaError: " + areaError);
		System.out.println("yawError: " + yawError);
		System.out.println("gyroError: " + gyroError);

		System.out.println("");

		System.out.println("vx: " + vx);
		System.out.println("vy: " + vy);
		System.out.println("vr: " + 0.0);

		System.out.println("");
		System.out.println("");

		ChassisSpeeds chassisSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, m_swerve.getGyroHeading());

		m_swerve.drive(chassisSpeeds);
	}

	@Override
	public boolean isFinished() {
		return m_atPosition;
	}

	@Override
	public void end(boolean interrupted) {
		m_limelight.setLEDs(true);
		m_swerve.setNeutralMode(NeutralMode.Coast);
	}
}
