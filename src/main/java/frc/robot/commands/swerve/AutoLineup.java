package frc.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoLineup extends CommandBase {
	private static final double TARGET_AREA = 0.3;
	private static final double TARGET_YAW = -3.0;
	private static final double TARGET_GYRO = Math.toRadians(0.0);

	private static final double ALLOWABLE_AREA_ERROR = 0.05;
	private static final double ALLOWABLE_YAW_ERROR = 1.0;
	private static final double ALLOWABLE_GYRO_ERROR = Math.toRadians(2.0);

	private final Swerve m_swerve;
	private final Limelight m_limelight;

	private final PIDController m_strafePIDController;
	private final PIDController m_distancePIDController;
	private final PIDController m_rotationPIDController;

	private double areaError = 1000.0;
	private double yawError = 1000.0;
	private double gyroError = 1000.0;

	public AutoLineup(Swerve swerve, Limelight limelight) {
		m_swerve = swerve;
		m_limelight = limelight;

		m_strafePIDController = new PIDController(0.075, 0.0, 0.0);
		m_distancePIDController = new PIDController(4.0, 0.0, 0.0);
		m_rotationPIDController = new PIDController(0.2, 0.0, 0.0);

		m_strafePIDController.setTolerance(ALLOWABLE_YAW_ERROR);
		m_distancePIDController.setTolerance(ALLOWABLE_AREA_ERROR);
		m_rotationPIDController.setTolerance(ALLOWABLE_GYRO_ERROR);

		addRequirements(swerve, limelight);
	}

	@Override
	public void initialize() {
		m_swerve.setNeutralMode(NeutralMode.Brake);
		m_limelight.setLEDs(true);
	}

	@Override
	public void execute() {
		m_limelight.setLEDs(true);

		PhotonTrackedTarget bestTarget = m_limelight.getBestTarget();

		if (bestTarget == null) {
			return;
		}

		areaError = TARGET_AREA - bestTarget.getArea();
		yawError = TARGET_YAW - bestTarget.getYaw();
		gyroError = TARGET_GYRO - m_swerve.getGyroHeading().getRadians();

		double vx = !(Math.abs(areaError) <= ALLOWABLE_AREA_ERROR) ? m_distancePIDController.calculate(areaError) : 0.0;
		double vy = !(Math.abs(yawError) <= ALLOWABLE_YAW_ERROR) ? m_strafePIDController.calculate(yawError) : 0.0;
		double vr = !(Math.abs(gyroError) <= ALLOWABLE_GYRO_ERROR) ? m_rotationPIDController.calculate(gyroError) : 0.0;

		ChassisSpeeds chassisSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
						vx,
						vy,
						vr,
						m_swerve.getGyroHeading());

		m_swerve.drive(chassisSpeeds);
	}

	@Override
	public boolean isFinished() {
		return (Math.abs(areaError) <= ALLOWABLE_AREA_ERROR
				&& Math.abs(yawError) <= ALLOWABLE_YAW_ERROR
				&& Math.abs(gyroError) <= ALLOWABLE_GYRO_ERROR);
	}

	@Override
	public void end(boolean interrupted) {
		m_limelight.setLEDs(true);
		m_swerve.setNeutralMode(NeutralMode.Coast);
	}
}
