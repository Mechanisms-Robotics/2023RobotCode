package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Limelight;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoLineup extends CommandBase {
	private static final double TARGET_AREA = 0.0;
	private static final double TARGET_YAW = 0.0;
	private static final double TARGET_GYRO = Math.toRadians(0.0);

	private static final double ALLOWABLE_AREA_ERROR = 0.0;
	private static final double ALLOWABLE_YAW_ERROR = 0.0;
	private static final double ALLOWABLE_GYRO_ERROR = 0.0;

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

		m_strafePIDController = new PIDController(0.0, 0.0, 0.0);
		m_distancePIDController = new PIDController(0.0, 0.0, 0.0);
		m_rotationPIDController = new PIDController(0.0, 0.0, 0.0);

		addRequirements(swerve, limelight);
	}

	@Override
	public void execute() {
		PhotonTrackedTarget bestTarget = m_limelight.getBestTarget();

		if (bestTarget == null) {
			return;
		}

		areaError = TARGET_AREA - bestTarget.getArea();
		yawError = TARGET_YAW - bestTarget.getYaw();
		gyroError = TARGET_GYRO - m_swerve.getGyroHeading().getRadians();

		ChassisSpeeds chassisSpeeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(
						m_distancePIDController.calculate(areaError),
						m_strafePIDController.calculate(yawError),
						m_rotationPIDController.calculate(gyroError),
						m_swerve.getGyroHeading());

		m_swerve.drive(chassisSpeeds);
	}

	@Override
	public boolean isFinished() {
		return (Math.abs(areaError) <= ALLOWABLE_AREA_ERROR
				&& Math.abs(yawError) <= ALLOWABLE_YAW_ERROR
				&& Math.abs(gyroError) <= ALLOWABLE_GYRO_ERROR);
	}
}
