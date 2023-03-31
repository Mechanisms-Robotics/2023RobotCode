package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;

public class ReleaseCommand extends SequentialCommandGroup {
	public ReleaseCommand(Superstructure superstructure) {
		super(
				new PrintCommand("--- CALLED ---"),
				new WaitUntilCommand(superstructure::atPosition),
				new WaitCommand(0.375),
				new InstantCommand(superstructure::open),
				new WaitCommand(0.1875),
				new InstantCommand(superstructure::idle),
				new PrintCommand("--- RELEASED ---"));
	}
}
