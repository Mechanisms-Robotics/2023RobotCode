package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class Superstructure extends SubsystemBase {

    private static final double[][] SPEEDS = new double[][] {
            {0.35, -0.75, 0.45},
            {0.40, -0.25, 0.45}
    };

    private enum State {
        Idle,
        Feed,
        Unjam
    }

    public enum Element {
        Cube(0), Cone(1);

        public final int index;

        Element(int i) {
            index = i;
        }
    }

    private final Intake intake;
    private final Feeder feeder;
    private final Conveyor conveyor;

    private State state = State.Idle;
    private Element element = Element.Cube;

    public Superstructure(Intake intake, Feeder feeder, Conveyor conveyor) {
        this.intake = intake;
        this.feeder = feeder;
        this.conveyor = conveyor;
    }

    @Override
    public void periodic() {
        switch (state) {
            case Idle:
                idle(); break;
            case Feed:
                feed(); break;
            case Unjam:
                unjam(); break;
        }

        if (!conveyor.conveyorSensor.get()) {
            conveyor.stop();
        }
    }

    public void setElement(Element element) {
        this.element = element;
    }

    public void feed() {
        state = State.Feed;
        intake.intake(SPEEDS[element.index][0]);
        feeder.feed(SPEEDS[element.index][1]);
        conveyor.convey(SPEEDS[element.index][2]);
    }

    public void unjam() {
        state = State.Unjam;
        intake.outtake();
        feeder.unjam();
        conveyor.unjam();
    }

    public void idle() {
        state = State.Idle;
        intake.stop();
        feeder.stop();
        conveyor.stop();
    }

}
