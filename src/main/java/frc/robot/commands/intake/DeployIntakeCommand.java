package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class DeployIntakeCommand extends InstantCommand {
  public DeployIntakeCommand(Intake intake) {
    super(intake::deploy);
  }
}
