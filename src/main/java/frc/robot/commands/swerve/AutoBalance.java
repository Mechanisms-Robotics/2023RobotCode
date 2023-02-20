package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/** Command to auto balance the robot on the switch. */
public class AutoBalance extends CommandBase {

	private static final double ALLOWABLE_PITCH_ERROR_DEGREES = 3.0;
	private static final double BASE_SPEED_METERS_PER_SEC = 0.1;
	private static final double VERIFY_BALANCED_DURATION_SEC = 1.5;

	private final Swerve swerve;
	private final Timer timer;
	private double pitchDegrees;
	//    private final ProfiledPIDController controller;
	//    private final TrapezoidProfile.Constraints constraints = new
	// TrapezoidProfile.Constraints(0.1, 0.1);

	public AutoBalance(Swerve swerve) {
		this.swerve = swerve;
		this.timer = new Timer();
		//        this.controller = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);
		//        controller.setTolerance(ALLOWABLE_PITCH_ERROR_DEGREES);

		addRequirements(swerve);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		pitchDegrees = swerve.getGyroPitch().getDegrees();

		// TODO: Check robot pose before starting to climb so we don't sit at the bottom of the
		// switch.

		if (isAboveUpperRange(pitchDegrees, 0.0, ALLOWABLE_PITCH_ERROR_DEGREES)) {
			swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, swerve.getGyroHeading()));
			timer.reset();
		} else if (isBelowLowerRange(pitchDegrees, 0.0, ALLOWABLE_PITCH_ERROR_DEGREES)) {
			swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							-BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, swerve.getGyroHeading()));
			timer.reset();
		} else {
			swerve.stop();
			timer.start();
		}
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(VERIFY_BALANCED_DURATION_SEC);
	}

	private boolean withinRange(double value, double reference, double range) {
		return !isBelowLowerRange(value, reference, range / 2)
				&& !isAboveUpperRange(value, reference, range / 2);
	}

	private boolean isBelowLowerRange(double value, double reference, double range) {
		return value < reference - range;
	}

	private boolean isAboveUpperRange(double value, double reference, double range) {
		return value > reference + range;
	}
}
