package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Superstructure;

public class OuttakeCommand extends FunctionalCommand {
	public OuttakeCommand(Superstructure superstructure) {
		super(
				superstructure::outtake,
				() -> {},
				(interrupted) -> {
					superstructure.idle();
				},
				() -> false,
				superstructure);
	}
}
