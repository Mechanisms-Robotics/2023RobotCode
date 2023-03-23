package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
	private static final double DEADBAND = 0.2;

	private static final double ANTI_TIP_SPEED = 0.75; // m/s

	private final Swerve m_swerveSubsystem;

	private final DoubleSupplier m_translationXSupplier;
	private final DoubleSupplier m_translationYSupplier;
	private final DoubleSupplier m_rotationSupplier;

	private final SlewRateLimiter m_vxRateLimiter =
			new SlewRateLimiter(
					4.5 // m/s^2
					);

	private final SlewRateLimiter m_vyRateLimiter =
			new SlewRateLimiter(
					4.5 // m/s^2
			);

	private final SlewRateLimiter m_rotRateLimiter =
			new SlewRateLimiter(
					2 * Math.PI // rads/s^2
					);

	public DriveCommand(
			Swerve swerveSubsystem,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier) {
		m_swerveSubsystem = swerveSubsystem;

		m_translationXSupplier = translationXSupplier;
		m_translationYSupplier = translationYSupplier;
		m_rotationSupplier = rotationSupplier;

		addRequirements(swerveSubsystem);
	}

	@Override
	public void execute() {
		double tilt =
				m_swerveSubsystem.getUpAngle().minus(m_swerveSubsystem.getRoll()).getDegrees();

		SmartDashboard.putNumber(
				"Tilt",
				m_swerveSubsystem.getUpAngle().minus(m_swerveSubsystem.getRoll()).getDegrees());

		if (Math.abs(tilt) >= 10.0
				&& !m_swerveSubsystem.getClimbMode()
				&& !DriverStation.isAutonomousEnabled()) {
			if (tilt > 0) {
				m_swerveSubsystem.drive(new ChassisSpeeds(0.0, ANTI_TIP_SPEED, 0.0));
			} else {
				m_swerveSubsystem.drive(new ChassisSpeeds(0.0, -ANTI_TIP_SPEED, 0.0));
			}
		} else {
			if (!DriverStation.isAutonomousEnabled()) {
				m_swerveSubsystem.drive(
						ChassisSpeeds.fromFieldRelativeSpeeds(
								m_vxRateLimiter.calculate(
										deadband(m_translationXSupplier.getAsDouble())),
								m_vyRateLimiter.calculate(
										deadband(m_translationYSupplier.getAsDouble())),
								m_rotRateLimiter.calculate(
										deadband(m_rotationSupplier.getAsDouble())),
								m_swerveSubsystem.getGyroHeading()));
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}

	private double deadband(double input) {
		return Math.abs(input) >= DEADBAND ? input : 0.0;
	}
}
