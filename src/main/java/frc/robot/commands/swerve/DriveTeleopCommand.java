package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class DriveTeleopCommand extends CommandBase {

	private static final double MAX_TRANSLATIONAL_VELOCITY_RATE = 4.0; // m/s per second
	private static final double MAX_ROTATION_VELOCITY_RATE = 2 * Math.PI; // rads/s per second

	// Joystick deadband
	private static final double DEADBAND = 0.1; // joystick percentage

	private final Supplier<Double> vxSupplier;
	private final Supplier<Double> vySupplier;
	private final Supplier<Double> vrSupplier;

	private final SlewRateLimiter vxRateLimiter;
	private final SlewRateLimiter vyRateLimiter;
	private final SlewRateLimiter vrRateLimiter;

	private final boolean fieldOriented;

	private final Swerve swerve;

	public DriveTeleopCommand(
			Supplier<Double> speedX,
			Supplier<Double> speedY,
			Supplier<Double> speedRotation,
			boolean fieldOriented,
			Swerve swerve) {
		this.vxSupplier = speedX;
		this.vySupplier = speedY;
		this.vrSupplier = speedRotation;

		this.fieldOriented = fieldOriented;

		this.vxRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
		this.vyRateLimiter = new SlewRateLimiter(MAX_TRANSLATIONAL_VELOCITY_RATE);
		this.vrRateLimiter = new SlewRateLimiter(MAX_ROTATION_VELOCITY_RATE);

		this.addRequirements(swerve);
		this.swerve = swerve;
	}

	@Override
	public void execute() {
		// Get the driver translation and rotation inputs then deadband them
		double translationX = deadband(vxSupplier.get());
		double translationY = deadband(vySupplier.get());
		double rotation = deadband(vrSupplier.get());

		// Scale the translational input
		Translation2d translation =
				scaleTranslationInput(new Translation2d(translationX, translationY));

		// Apply Ramp Rates
		translationX = vxRateLimiter.calculate(translation.getY() * Swerve.maxVelocity);
		translationY = vyRateLimiter.calculate(translation.getX() * Swerve.maxVelocity);
		rotation = vrRateLimiter.calculate(rotation * Swerve.maxRotationalVelocity);

		// Drive the swerve
		swerve.drive(translationX, translationY, rotation, fieldOriented);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	/**
	 * Scales translational inputs to an exponential curve
	 *
	 * @param input Translational input
	 * @return Scaled translational input
	 */
	private Translation2d scaleTranslationInput(Translation2d input) {
		// Get the magnitude of the translation
		double mag = input.getNorm();

		// Scale it to an exponential
		final double scale = 1.35;
		mag = Math.pow(mag, scale);

		// Multiply the translational input with the exponential, and return it
		final Rotation2d rotation = new Rotation2d(input.getX(), input.getY());
		return new Translation2d(rotation.getCos() * mag, rotation.getSin() * mag);
	}

	private static double deadband(double input) {
		return Math.abs(input) > DEADBAND ? input : 0;
	}
}
