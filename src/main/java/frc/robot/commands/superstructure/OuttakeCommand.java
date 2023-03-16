package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

public class OuttakeCommand extends InstantCommand {
	public OuttakeCommand(Superstructure superstructure) {
		super(
				superstructure::outtake,
				superstructure);
	}
}
