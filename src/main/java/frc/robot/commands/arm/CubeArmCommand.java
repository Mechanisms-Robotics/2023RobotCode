package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

import java.util.function.BooleanSupplier;

public class CubeArmCommand extends CommandBase {

    private enum State {
        Backstopping,
        GoingToPosition,
    }

    private final Arm arm;
    private final Gripper gripper;

    private final BooleanSupplier conveyorSensor;

    private State state = State.Backstopping;
    private boolean grabbed = false;

    public CubeArmCommand(Arm arm, Gripper gripper, BooleanSupplier conveyorSensor) {
        this.arm = arm;
        this.gripper = gripper;
        this.conveyorSensor = conveyorSensor;

        addRequirements(arm, gripper);
    }

    @Override
    public void initialize() {
        state = State.Backstopping;
        grabbed = false;
        arm.cubeBackStop();
    }

    @Override
    public void execute() {

                    System.out.println(state);
        switch (state) {
            case Backstopping:
                gripper.close();
                if (!conveyorSensor.getAsBoolean()) {
                    arm.cubeGrabPosition();
                    state = State.GoingToPosition;
                }
                break;
            case GoingToPosition:
                gripper.open();
                if (arm.isAtPosition()) {
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
