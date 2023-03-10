package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
	private static final double DEADBAND = 0.1;
	private static final double TRANSLATION_EXPONENT = 1.5;
	private static final double ROTATION_EXPONENT = 2.0;

	private static final double ANTI_TIP_SPEED = 0.75; // m/s

	private final Swerve m_swerveSubsystem;

	private final DoubleSupplier m_translationXSupplier;
	private final DoubleSupplier m_translationYSupplier;
	private final DoubleSupplier m_rotationSupplier;
	private final BooleanSupplier m_isFieldRelativeSupplier;
	private final BooleanSupplier m_isOpenLoopSupplier;

	public DriveCommand(
			Swerve swerveSubsystem,
			DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier,
			BooleanSupplier isFieldRelativeSupplier,
			BooleanSupplier isOpenLoopSupplier) {
		m_swerveSubsystem = swerveSubsystem;

		m_translationXSupplier = translationXSupplier;
		m_translationYSupplier = translationYSupplier;
		m_rotationSupplier = rotationSupplier;
		m_isFieldRelativeSupplier = isFieldRelativeSupplier;
		m_isOpenLoopSupplier = isOpenLoopSupplier;

		addRequirements(swerveSubsystem);
	}

	@Override
	public void execute() {
		double tilt = m_swerveSubsystem.getUpAngle().minus(m_swerveSubsystem.getRoll()).getDegrees();

		SmartDashboard.putNumber("Tilt", m_swerveSubsystem.getUpAngle().minus(m_swerveSubsystem.getRoll()).getDegrees());

		if (Math.abs(tilt) >= 10.0) {
			if (tilt > 0) {
				m_swerveSubsystem.drive(new Translation2d(
						0.0,
						ANTI_TIP_SPEED),
						0.0,
						m_isFieldRelativeSupplier.getAsBoolean(),
						m_isOpenLoopSupplier.getAsBoolean()
				);
			} else {
				m_swerveSubsystem.drive(new Translation2d(
								0.0,
								-ANTI_TIP_SPEED),
						0.0,
						m_isFieldRelativeSupplier.getAsBoolean(),
						m_isOpenLoopSupplier.getAsBoolean()
				);
			}
    } else {
//      m_swerveSubsystem.drive(
//          ChassisSpeeds.fromFieldRelativeSpeeds(
//              applyExponential(
//                  deadband(m_translationXSupplier.getAsDouble()), TRANSLATION_EXPONENT),
//              applyExponential(
//                  deadband(m_translationYSupplier.getAsDouble()), TRANSLATION_EXPONENT),
//              applyExponential(deadband(m_rotationSupplier.getAsDouble()), ROTATION_EXPONENT),
//              m_swerveSubsystem.getYaw()));

			m_swerveSubsystem.drive(
					new Translation2d(
							applyExponential(deadband(m_translationXSupplier.getAsDouble()), TRANSLATION_EXPONENT),
							applyExponential(deadband(m_translationYSupplier.getAsDouble()), TRANSLATION_EXPONENT)
					),
					applyExponential(deadband(m_rotationSupplier.getAsDouble()), ROTATION_EXPONENT),
					m_isFieldRelativeSupplier.getAsBoolean(),
					m_isOpenLoopSupplier.getAsBoolean()
					);

		}

//		System.out.println(
//				applyExponential(
//						deadband(m_translationXSupplier.getAsDouble()), TRANSLATION_EXPONENT));
	}

	@Override
	public void end(boolean interrupted) {
		m_swerveSubsystem.stop();
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
