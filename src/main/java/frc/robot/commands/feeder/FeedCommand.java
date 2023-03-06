package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FeedCommand extends CommandBase {

    private static final double FEEDER_CUBE_SPEED = -0.75;
    private static final double FEEDER_CONE_SPEED = -0.25;

    private static final double CONVEYOR_CUBE_SPEED = 0.45;
    private static final double CONVEYOR_CONE_SPEED = 0.45;

    private final Feeder feeder;
    private final Conveyor conveyor;

    private final BooleanSupplier isCube;

    public FeedCommand(Feeder feeder, Conveyor conveyor, BooleanSupplier isCube) {
        this.feeder = feeder;
        this.conveyor = conveyor;
        this.isCube = isCube;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        feeder.feed(isCube.getAsBoolean() ? FEEDER_CUBE_SPEED : FEEDER_CONE_SPEED);
        conveyor.convey(isCube.getAsBoolean() ? CONVEYOR_CUBE_SPEED : CONVEYOR_CONE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
