package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class TornadoCommand extends CommandBase {

	private final Swerve swerve;

	private static final ChassisSpeeds TORNADO_MODE = new ChassisSpeeds(0.0, 0.0, 2 * Math.PI);

	public TornadoCommand(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		System.out.println("Overriding max_velocity...");
	}

	@Override
	public void execute() {
		swerve.drive(TORNADO_MODE);
		System.out.println("TORNADOOOOOOOOOO");
	}
}
