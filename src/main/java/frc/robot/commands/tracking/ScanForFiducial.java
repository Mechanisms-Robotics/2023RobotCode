package frc.robot.commands.tracking;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AprilTagTracker;
import java.util.Optional;

public class ScanForFiducial extends CommandBase {

	private final Swerve swerve;
	private boolean foundTag = false;
	private Optional<Integer> fiduicialId = Optional.empty();

	public ScanForFiducial(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	public ScanForFiducial(Swerve swerve, int id) {
		this(swerve);
		this.fiduicialId = Optional.of(id);
	}

	@Override
	public void execute() {
		boolean idExist = fiduicialId.isPresent();
		foundTag = AprilTagTracker.getBestTarget().isPresent();

		if (!foundTag) {
			swerve.drive(
					ChassisSpeeds.fromFieldRelativeSpeeds(
							0.0, 0.0, Math.PI / 3, swerve.getGyroHeading()));
		}
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return foundTag;
	}
}
