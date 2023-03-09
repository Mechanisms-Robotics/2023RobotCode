package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

	private static final double[][] SPEEDS =
			new double[][] {
				{0.30, -1.0, 0.45, 0.2}, // Cube
				{0.40, -0.25, 0.45, 0.2} // Cone
			};

	private enum State {
		Idle,
		Feed,
		Positioning,
		Unjam
	}

	public enum Element {
		Cube(0),
		Cone(1);

		public final int index;

		Element(int i) {
			index = i;
		}
	}

	public enum Level {
		Low,
		Mid,
		High,
	}

	private final Intake intake;
	private final Feeder feeder;
	private final Conveyor conveyor;
	private final Arm arm;
	private final Gripper gripper;

	private State state = State.Idle;
	private Element element = Element.Cube;
	private Level level = Level.High;

	private boolean isPositioned = false;

	public Superstructure(
			Intake intake, Feeder feeder, Conveyor conveyor, Arm arm, Gripper gripper) {
		this.intake = intake;
		this.feeder = feeder;
		this.conveyor = conveyor;
		this.arm = arm;
		this.gripper = gripper;
	}

	@Override
	public void periodic() {
		switch (state) {
			case Idle:
				idle();
				break;
			case Feed:
				feed();
				break;
			case Positioning:
				position();
				break;
			case Unjam:
				unjam();
				break;
		}
	}

	public void setElement(Element element) {
		this.element = element;
	}

	public void setLevel(Level level) {
		this.level = level;
	}

	public Element getElement() {
		return this.element;
	}

	public Level getLevel() {
		return this.level;
	}

	public void feed() {
		isPositioned = false;

		if (!conveyor.getDebouncedSensor()) {
			position();
			return;
		}

		state = State.Feed;
		intake.intake(SPEEDS[element.index][0]);
		feeder.feed(SPEEDS[element.index][1]);
		conveyor.convey(SPEEDS[element.index][2]);
	}

	public void position() {
		if (conveyor.getDebouncedSensor()) {
			isPositioned = true;

			idle();
			return;
		}

		state = State.Positioning;
		intake.intake(SPEEDS[element.index][0]);
		feeder.stop();
		conveyor.convey(SPEEDS[element.index][3]);
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

	public boolean isPositioned() {
		return isPositioned;
	}
}
