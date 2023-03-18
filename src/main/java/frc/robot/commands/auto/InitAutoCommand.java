package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class InitAutoCommand extends InstantCommand {
  public InitAutoCommand(Swerve swerve, Intake intake, Arm arm, Gripper gripper) {
    super(() -> {
      swerve.setNeutralMode(NeutralMode.Brake);
      swerve.zeroGyro();

      swerve.resetModules();

      intake.zeroEncoders();
      arm.init();
      gripper.zeroEncoder();
    });
  }
}
