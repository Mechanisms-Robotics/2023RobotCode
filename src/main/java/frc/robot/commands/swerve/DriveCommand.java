package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
	private static final double DEADBAND = 0.1;
	private static final double TRANSLATION_EXPONENT = 1.5;
	private static final double ROTATION_EXPONENT = 2.0;

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
		//		m_swerveSubsystem.drive(
		//				new ChassisSpeeds(
		//						applyExponential(
		//								deadband(m_translationXSupplier.getAsDouble()),
		//								TRANSLATION_EXPONENT),
		//						applyExponential(
		//										deadband(m_translationYSupplier.getAsDouble()),
		//										TRANSLATION_EXPONENT)
		//								/ 2,
		//						applyExponential(
		//								deadband(m_rotationSupplier.getAsDouble()), ROTATION_EXPONENT)));
		m_swerveSubsystem.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						applyExponential(
								deadband(m_translationXSupplier.getAsDouble()),
								TRANSLATION_EXPONENT),
						applyExponential(
								deadband(m_translationYSupplier.getAsDouble()),
								TRANSLATION_EXPONENT),
						applyExponential(
								deadband(m_rotationSupplier.getAsDouble()), ROTATION_EXPONENT),
						m_swerveSubsystem.getGyroHeading()));

		System.out.println(
				applyExponential(
						deadband(m_translationXSupplier.getAsDouble()), TRANSLATION_EXPONENT));
	}

	@Override
	public void end(boolean interrupted) {
		m_swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}

	private double deadband(double input) {
		return Math.abs(input) >= DEADBAND ? input : 0.0;
	}

	private double applyExponential(double input, double exponent) {
		double product = Math.pow(Math.abs(input), exponent);
		return input > 0 ? product : -product;
	}

	//	private double desaturateXSpeeds(double xSpeed) {
	//		return 1 / m_swerveSubsystem.getGyroHeading().getCos() * XSPEED_DESATURATION;
	//	}
}
