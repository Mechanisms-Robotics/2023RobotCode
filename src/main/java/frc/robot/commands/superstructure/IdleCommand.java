package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

public class IdleCommand extends InstantCommand {
  public IdleCommand(Superstructure m_superstructure) {
    super(m_superstructure::idle, m_superstructure);
  }
}
