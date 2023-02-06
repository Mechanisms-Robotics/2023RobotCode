package frc.robot.commands.tracking;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AprilTagTracker;
import java.util.List;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;

public class MoveRelativeToFiducial extends CommandBase {

	private final Swerve swerve;

	private final ProfiledPIDController thetaController;
	private static final TrapezoidProfile.Constraints HEADING_CONSTRAINTS =
			new TrapezoidProfile.Constraints(2 * Math.PI, 4 * Math.PI);
	private final HolonomicDriveController driveController;

	private static final double X_KP = 1.0;
	private static final double Y_KP = X_KP;

	private static final int MAX_ITERATION_ATTEMPTS = 10;

	private static final double ALLOWABLE_POS_ERROR = 0.1; // m
	private static final double ALLOWABLE_ROT_ERROR = 1; // deg

	private int noTargetsFoundCount;
	private boolean shouldEnd = false;

	public MoveRelativeToFiducial(Swerve swerve) {
		this.swerve = swerve;

		thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, HEADING_CONSTRAINTS);
		driveController =
				new HolonomicDriveController(
						new PIDController(X_KP, 0.0, 0.0),
						new PIDController(Y_KP, 0.0, 0.0),
						thetaController);

		driveController.setTolerance(
				new Pose2d(
						new Translation2d(ALLOWABLE_POS_ERROR, ALLOWABLE_POS_ERROR),
						Rotation2d.fromDegrees(ALLOWABLE_ROT_ERROR)));

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		List<PhotonTrackedTarget> targets = AprilTagTracker.getTargets();

		if (targets == null || noTargetsFoundCount >= MAX_ITERATION_ATTEMPTS) {
			shouldEnd = true;
			noTargetsFoundCount++;
			return;
		}

		targets.sort(PhotonTargetSortMode.Largest.getComparator());
		int id = targets.get(0).getFiducialId();
		Pose3d tagPose = AprilTagTracker.getFieldLayout().getTagPose(id).get();
		Rotation2d desiredHeading = tagPose.getRotation().toRotation2d();
		ChassisSpeeds outputSpeeds =
				driveController.calculate(
						swerve.getPose(),
						new Trajectory.State(
								0.0,
								0.0,
								0.0,
								new Pose2d(
										5.0, 5.0, Rotation2d.fromDegrees(0)), // tagPose.toPose2d(),
								0.0),
						desiredHeading);
		swerve.drive(outputSpeeds);

		SmartDashboard.putNumber("X", outputSpeeds.vxMetersPerSecond);
		SmartDashboard.putNumber("Y", outputSpeeds.vyMetersPerSecond);
		SmartDashboard.putNumber("Rot", outputSpeeds.omegaRadiansPerSecond);
	}

	@Override
	public boolean isFinished() {
		return shouldEnd;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("No tags found to auto-align.");
	}
}
