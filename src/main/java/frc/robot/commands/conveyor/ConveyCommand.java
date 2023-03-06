package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ConveyCommand extends CommandBase {

    private static final double TIME_TO_STOP = 0.2;

    private final Timer timer = new Timer();
    private final Conveyor conveyor;
    private final BooleanSupplier conveyorSensorSupplier;

    public ConveyCommand(BooleanSupplier conveyorSensorResult, Conveyor conveyor) {
        this.conveyor = conveyor;
        this.conveyorSensorSupplier = conveyorSensorResult;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.convey(0.0);
        System.out.println("HABABABABBABABABBABDBADBABDBA");
    }

    @Override
    public void execute() {
        if (!conveyorSensorSupplier.getAsBoolean()) {
            timer.start();
            System.out.println("START TIMER");
        }
        if (timer.hasElapsed(TIME_TO_STOP)) {
            System.out.println("STOP");
            conveyor.stop();
            timer.stop();
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        System.out.println("stop");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
