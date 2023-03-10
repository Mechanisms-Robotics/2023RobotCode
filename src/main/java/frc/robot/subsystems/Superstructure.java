package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

	private static final double[][] INTAKE_SPEEDS =
			new double[][] {
				{0.30, -1.0, 0.45, 0.2}, // Intake, Feeder, Conveyor, Positioning | Cube
				{0.40, -0.25, 0.45, 0.2} // Intake, Feeder, Conveyor, Positioning | Cone
			};

	private static final double[][] OUTTAKE_SPEEDS =
			new double[][] {
				{-0.15, 0.1, -0.3}, // Intake, Feeder, Conveyor | Cube
				{-0.15, 0.1, -0.3} // Intake, Feeder, Conveyor | Cone
			};

	private static final double[][] ARM_POSITIONS =
			new double[][] {
				{
					17500, 21500, 16875, 30000, 52500, 57500
				}, // Idling, Backstopping, Grabbing, Low, Mid, High | Cube
				{
					17500, 21500, 16875, 30000, 52500, 57500
				} // Idling, Backstopping, Grabbing, Low, Mid, High | Cone
			};

	private static final double[][] EXTENSION_POSITIONS =
			new double[][] {
				{
					-500, -500, -250, -500, -8700, -17400
				}, // Idling, Backstopping, Grabbing, Low, Mid, High | Cube
				{
					-500, -500, -250, -500, -8700, -17400
				} // Idling, Backstopping, Grabbing, Low, Mid, High | Cone
			};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{-50, -3765, -3000}, // Idling, Backstopping, Grabbing | Cube
				{-50, -50, -3765} // Idling, Backstopping, Grabbing | Cone
			};

	private static final double SCORE_TIME = 0.5; // seconds

	private enum IntakeState {
		Idling,
		Intaking,
		Positioning,
		Outtaking
	}

	private enum ArmState {
		Idling,
		Backstopping,
		Grabbing,
		Holding,
		Prepping,
		Scoring
	}

	private enum State {
		Idling,
		Intaking,
		Outtaking,
		Holding,
		Prepping,
		Scoring
	}

	public enum Element {
		Cube(0),
		Cone(1);

		public final int index;

		Element(int i) {
			index = i;
		}
	}

	private final Intake m_intake;
	private final Feeder m_feeder;
	private final Conveyor m_conveyor;
	private final Arm m_arm;
	private final Gripper m_gripper;

	private IntakeState m_intakeState = IntakeState.Idling;
	private ArmState m_armState = ArmState.Idling;

	private State m_state = State.Idling;

	private Element m_element = Element.Cube;

	private int[] m_targetNode = {0, 0};

	private final Timer m_scoreTimer = new Timer();

	public Superstructure(
			Intake intake, Feeder feeder, Conveyor conveyor, Arm arm, Gripper gripper) {
		m_intake = intake;
		m_feeder = feeder;
		m_conveyor = conveyor;
		m_arm = arm;
		m_gripper = gripper;
	}

	@Override
	public void periodic() {
		switch (m_state) {
			case Idling:
				idle();
				break;
			case Intaking:
				intake();
				break;
			case Outtaking:
				outtake();
				break;
			case Holding:
				hold();
				break;
			case Prepping:
				prep();
				break;
			case Scoring:
				score();
				break;
		}

		SmartDashboard.putString("State", m_state.toString());
		SmartDashboard.putString("Element", m_element.toString());
	}

	@Override
	public void simulationPeriodic() {
		super.simulationPeriodic();
		periodic();
	}

	public void idle() {
		m_intakeState = IntakeState.Idling;
		m_armState = ArmState.Idling;

		m_state = State.Idling;

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		m_arm.setArm(ARM_POSITIONS[m_element.index][0], EXTENSION_POSITIONS[m_element.index][0]);

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
	}

	public void intake() {
		if (m_intakeState != IntakeState.Intaking
				&& m_intakeState != IntakeState.Positioning
				&& m_intakeState != IntakeState.Outtaking) m_intakeState = IntakeState.Intaking;

		if (m_armState != ArmState.Backstopping) m_armState = ArmState.Backstopping;

		if (m_state != State.Intaking) m_state = State.Intaking;

		if (m_intakeState == IntakeState.Intaking) {
			if (m_conveyor.getDebouncedSensor(0)) {
				m_intakeState = IntakeState.Positioning;
				m_armState = ArmState.Grabbing;

				return;
			}

			m_intake.intake(INTAKE_SPEEDS[m_element.index][0]);
			m_feeder.feed(INTAKE_SPEEDS[m_element.index][1]);
			m_conveyor.convey(INTAKE_SPEEDS[m_element.index][2]);

			m_arm.setArm(
					ARM_POSITIONS[m_element.index][1], EXTENSION_POSITIONS[m_element.index][1]);

			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][1]);
		} else if (m_intakeState == IntakeState.Positioning) {
			if (m_conveyor.getDebouncedSensor(1)) {
				hold();
				return;
			}

			m_intake.stop();
			m_feeder.stop();
			m_conveyor.convey(INTAKE_SPEEDS[m_element.index][3]);

			m_arm.setArm(
					ARM_POSITIONS[m_element.index][2], EXTENSION_POSITIONS[m_element.index][2]);

			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
		}
	}

	public void outtake() {
		if (m_intakeState != IntakeState.Outtaking) m_intakeState = IntakeState.Outtaking;
		if (m_armState != ArmState.Idling) m_armState = ArmState.Idling;

		if (m_state != State.Outtaking) m_state = State.Outtaking;

		m_intake.outtake(OUTTAKE_SPEEDS[m_element.index][0]);
		m_feeder.outtake(OUTTAKE_SPEEDS[m_element.index][1]);
		m_conveyor.outtake(OUTTAKE_SPEEDS[m_element.index][2]);

		m_arm.setArm(ARM_POSITIONS[m_element.index][0], EXTENSION_POSITIONS[m_element.index][0]);

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
	}

	public void hold() {
		if (m_intakeState != IntakeState.Idling) m_intakeState = IntakeState.Idling;
		if (m_armState != ArmState.Holding) m_armState = ArmState.Holding;

		if (m_state != State.Holding) m_state = State.Holding;

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		m_arm.setArm(ARM_POSITIONS[m_element.index][2], EXTENSION_POSITIONS[m_element.index][2]);

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][2]);
	}

	public void prep() {
		if (m_intakeState != IntakeState.Idling) m_intakeState = IntakeState.Idling;
		if (m_armState != ArmState.Prepping) m_armState = ArmState.Prepping;

		if (m_state != State.Prepping) m_state = State.Prepping;

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		m_arm.setArm(
				ARM_POSITIONS[m_element.index][3 + m_targetNode[0]],
				ARM_POSITIONS[m_element.index][3 + m_targetNode[0]]);

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][2]);
	}

	public void score() {
		if (m_intakeState != IntakeState.Idling) m_intakeState = IntakeState.Idling;
		if (m_armState != ArmState.Scoring) m_armState = ArmState.Scoring;

		if (m_state != State.Scoring) m_state = State.Scoring;

		if (!m_scoreTimer.hasElapsed(0.01)) {
			m_scoreTimer.start();
		}

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		m_arm.setArm(
				ARM_POSITIONS[m_element.index][3 + m_targetNode[0]],
				ARM_POSITIONS[m_element.index][3 + m_targetNode[0]]);

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);

		if (m_scoreTimer.hasElapsed(SCORE_TIME)) {
			idle();
		}
	}

	public void setElement(Element element) {
		m_element = element;
	}

	public void setNode(int row, int col) {
		m_targetNode[0] = row;
		m_targetNode[1] = col;
	}

	public boolean atPosition() {
		return m_arm.isIdle() && m_arm.isAtPosition() && m_arm.extendAtPosition();
	}
}
