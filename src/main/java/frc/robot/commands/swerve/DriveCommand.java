package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
	private final Swerve m_swerveSubsystem;

	private final DoubleSupplier m_translationXSupplier;
	private final DoubleSupplier m_translationYSupplier;
	private final DoubleSupplier m_rotationSupplier;

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
		m_swerveSubsystem.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						m_translationXSupplier.getAsDouble(),
						m_translationYSupplier.getAsDouble(),
						m_rotationSupplier.getAsDouble(),
						m_swerveSubsystem.getGyroHeading()));
	}

	@Override
	public void end(boolean interrupted) {
		m_swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}
