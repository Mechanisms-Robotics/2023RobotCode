package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class LockCommand extends InstantCommand {
  public LockCommand(Swerve swerve) {
    super(swerve::lock);
  }
}
