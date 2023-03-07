package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class DriveCommand extends CommandBase {
	private final Swerve swerve;
	private DoubleSupplier vX, vY, headingX, headingY, vR;
	private final boolean isOpenLoop;
	private final boolean relativeRotation;

	/**
	 * Used to drive a swerve robot in full field-centric mode. vX and vY supply translation inputs,
	 * where x is torwards/away from alliance wall and y is left/right. headingHorzontal and
	 * headingVertical are the Cartesian coordinates from which the robot's angle will be derived—
	 * they will be converted to a polar angle, which the robot will rotate to.
	 *
	 * @param swerve The swerve drivebase subsystem.
	 * @param vX DoubleSupplier that supplies the x-translation joystick input. Should be in the
	 *     range -1 to 1 with deadband already accounted for. Positive X is away from the alliance
	 *     wall.
	 * @param vY DoubleSupplier that supplies the y-translation joystick input. Should be in the
	 *     range -1 to 1 with deadband already accounted for. Positive Y is towards the left wall
	 *     when looking through the driver station glass.
	 * @param heading DoubleSupplier that supplies the robot's heading angle.
	 */
	public DriveCommand(
			Swerve swerve,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier headingX,
			DoubleSupplier headingY,
			boolean isOpenLoop) {
		this.swerve = swerve;
		this.vX = vX;
		this.vY = vY;
		this.headingX = headingX;
		this.headingY = headingY;
		this.isOpenLoop = isOpenLoop;
		this.relativeRotation = false;

		addRequirements(swerve);
	}

	public DriveCommand(
			Swerve swerve,
			DoubleSupplier vX,
			DoubleSupplier vY,
			DoubleSupplier vR,
			boolean isOpenLoop) {
		this.swerve = swerve;
		this.vX = vX;
		this.vY = vY;
		this.vR = vR;
		this.isOpenLoop = isOpenLoop;
		this.relativeRotation = true;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		// Get the desired chassis speeds based on a 2 joystick module.

		//		ChassisSpeeds desiredSpeeds =
		//				swerve.getTargetSpeeds(
		//						vX.getAsDouble(),
		//						vY.getAsDouble(),
		//						new Rotation2d(heading.getAsDouble() * Math.PI));

		ChassisSpeeds desiredSpeeds;

		if (relativeRotation) {
			desiredSpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							vX.getAsDouble(),
							vY.getAsDouble(),
							vR.getAsDouble(),
							swerve.getHeading());
		} else {
			desiredSpeeds =
					swerve.getTargetSpeeds(
							vX.getAsDouble(),
							vY.getAsDouble(),
							headingX.getAsDouble(),
							headingY.getAsDouble());
		}

		// Limit velocity to prevent tippy
		Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
//		translation =
//				SwerveMath.limitVelocity(
//						translation,
//						swerve.getFieldVelocity(),
//						swerve.getPose(),
//						Constants.LOOP_TIME,
//						Constants.ROBOT_MASS,
//						List.of(Constants.CHASSIS),
//						swerve.getSwerveDriveConfiguration());
		SmartDashboard.putNumber("LimitedTranslation", translation.getX());
		SmartDashboard.putString("Translation", translation.toString());

		// Make the robot move
		swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
