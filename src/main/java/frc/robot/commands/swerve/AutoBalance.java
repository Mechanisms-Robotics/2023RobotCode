package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/** Command to auto balance the robot on the switch. */
public class AutoBalance extends CommandBase {

	private static final double ALLOWABLE_PITCH_ERROR_DEGREES = 8.25;
	private static final double BASE_SPEED_METERS_PER_SEC = -0.25;
	private static final double VERIFY_BALANCED_DURATION_SEC = 0.15;

	private final Swerve swerve;
	private final Timer timer;

	public AutoBalance(Swerve swerve) {
		this.swerve = swerve;
		this.timer = new Timer();

		addRequirements(swerve);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double pitchDegrees = swerve.getPitch().getDegrees();
		double error = pitchDegrees;

    System.out.println("pitch: " + pitchDegrees);

		if (error > ALLOWABLE_PITCH_ERROR_DEGREES) {
			swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, swerve.getGyroHeading()));
			timer.stop();
			timer.reset();
		} else if (error < -ALLOWABLE_PITCH_ERROR_DEGREES) {
			swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							-BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, swerve.getGyroHeading()));
			timer.stop();
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
}
