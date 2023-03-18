package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

public class ShootOutCommand extends InstantCommand {
	public ShootOutCommand(Superstructure m_superstructure) {
		super(m_superstructure::shootOut);
	}
}
