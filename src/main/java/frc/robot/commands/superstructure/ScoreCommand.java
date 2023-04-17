package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Element;

public class ScoreCommand extends SequentialCommandGroup {
	public ScoreCommand(Superstructure superstructure, Element element, int level) {
		super(
				new InstantCommand(() -> superstructure.setElement(element)),
				new InstantCommand(() -> superstructure.setNode(level, 0)),
				new InstantCommand(superstructure::idle),
				new WaitUntilCommand(superstructure::atPosition).withTimeout(0.5),
				new InstantCommand(superstructure::close),
				new WaitUntilCommand(superstructure::atPosition).withTimeout(0.25),
				new InstantCommand(superstructure::score));
	}
}
