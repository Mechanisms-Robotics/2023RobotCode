package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;

public class ReleaseCommand extends SequentialCommandGroup {
	public ReleaseCommand(Superstructure superstructure) {
		super(
				new WaitUntilCommand(superstructure::atPosition).withTimeout(3.0),
				new WaitCommand(1.0),
				new InstantCommand(superstructure::open),
				new WaitCommand(0.25),
				new InstantCommand(superstructure::idle));
	}
}
