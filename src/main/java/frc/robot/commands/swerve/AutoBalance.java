package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/** Command to auto balance the robot on the switch. */
public class AutoBalance extends CommandBase {

	private static final double ALLOWABLE_PITCH_ERROR_DEGREES = 6.0; // 6.5
	private static final double BASE_SPEED_METERS_PER_SEC = -0.35;
	private static final double VERIFY_BALANCED_DURATION_SEC = 0.25;

	private static final double BALANCE_PITCH_THRESHOLD = 0.5;

	private final Swerve m_swerve;
	private final Timer m_timer;
	private final boolean m_isOnCloseSide;

	private double m_prevError;
	private boolean finished = false;

	public AutoBalance(Swerve swerve, boolean isOnCloseSide) {
		m_swerve = swerve;
		m_timer = new Timer();
		m_isOnCloseSide = isOnCloseSide;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		double pitchDegrees = m_swerve.getPitch().getDegrees();
		double error = pitchDegrees;


		double dt = pitchDt(error);
		System.out.println("error: " + error + " dt: " + dt);

		if (Math.abs(error) > ALLOWABLE_PITCH_ERROR_DEGREES && dt <= BALANCE_PITCH_THRESHOLD) {
			System.out.println("BBBB");
			m_swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, m_swerve.getGyroHeading()));
		} else if (Math.abs(error) <= ALLOWABLE_PITCH_ERROR_DEGREES && dt > BALANCE_PITCH_THRESHOLD){
			System.out.println("CCCC");
			finished = true;
			m_swerve.stop();
		}

    if (error > ALLOWABLE_PITCH_ERROR_DEGREES || error < -ALLOWABLE_PITCH_ERROR_DEGREES)
      System.out.println("---- ALLOWABLE PITCH-----------");

		m_prevError = error;
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted) System.out.println("---- AUTOBALACNE FINISHED ----");

		m_swerve.lock();
		m_swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return finished;
	}

	// Calculate change in pitch
	private double pitchDt(double currentPitch) {
		double dt = m_prevError - currentPitch;
		return Math.abs(dt) > 1.0 ? 0.0 : dt;
	}
}
