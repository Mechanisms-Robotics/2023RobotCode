package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

import frc.robot.subsystems.Intake;
import java.util.function.BooleanSupplier;

public class CubeArmCommand extends CommandBase {

    private enum State {
        Backstopping,
        GoingToPosition,
    }

    private final Arm arm;
    private final Gripper gripper;
    private final Intake intake;

    private final BooleanSupplier conveyorSensor;
    private final BooleanSupplier isPositioned;

    private State state = State.Backstopping;
    private boolean grabbed = false;

    public CubeArmCommand(Intake intake, Arm arm, Gripper gripper, BooleanSupplier conveyorSensor, BooleanSupplier isPositioned) {
        this.arm = arm;
        this.gripper = gripper;
        this.intake = intake;
        this.conveyorSensor = conveyorSensor;
        this.isPositioned = isPositioned;

        addRequirements(arm, gripper);
    }

    @Override
    public void initialize() {
        state = State.Backstopping;
        grabbed = false;
        arm.stow();
    }

    @Override
    public void execute() {
        switch (state) {
            case Backstopping:
                gripper.cone();
                if (!conveyorSensor.getAsBoolean()) {
                    arm.cubeGrabPosition();
                    state = State.GoingToPosition;
                    intake.stop();
                }
                break;
            case GoingToPosition:
                gripper.open();
                if (arm.isAtPosition() && isPositioned.getAsBoolean()) {
                    gripper.cube();
                    grabbed = true;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return grabbed;
    }
}
