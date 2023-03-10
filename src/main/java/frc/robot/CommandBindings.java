package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public final class CommandBindings {
  public InstantCommand m_deployIntakeCommand;
  public InstantCommand m_retractIntakeCommand;

 public CommandBindings(Intake intake) {
   m_deployIntakeCommand = new InstantCommand(
       intake::deploy
   );

   m_retractIntakeCommand = new InstantCommand(
       intake::retract
   );
 }
}
