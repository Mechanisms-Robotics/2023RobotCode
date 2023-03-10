package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class HPStationIntakeCommand extends InstantCommand {
  public HPStationIntakeCommand(Intake intake) {
    super(intake::hpStation);
  }
}
