package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class NewAutoBalance extends CommandBase {

	private static final double BASE_SPEED_METERS_PER_SEC = -0.35;
	private static final double ALLOWABLE_PITCH_ERROR_DEGREES = 6.0; // 6.5

	private static final double FORWARD_PITCH = 3.0;

	private final Swerve m_swerve;
	private final Timer m_timer;
	private final boolean m_isOnCloseSide;

	private boolean finished = false;

	public NewAutoBalance(Swerve swerve, boolean isOnCloseSide) {
		m_swerve = swerve;
		m_timer = new Timer();
		m_isOnCloseSide = isOnCloseSide;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		double pitch = m_swerve.getPitch().getDegrees();

		if (pitch >= FORWARD_PITCH) {
			m_swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, m_swerve.getGyroHeading()));
		} else if (pitch <= FORWARD_PITCH) {
			m_swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							-BASE_SPEED_METERS_PER_SEC, 0.0, 0.0, m_swerve.getGyroHeading()));
		} else if (Math.abs(pitch - FORWARD_PITCH) <= 0.2) {
			finished = true;
			m_swerve.stop();
		}
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
}
