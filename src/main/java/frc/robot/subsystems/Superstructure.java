package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDWrapper;

public class Superstructure extends SubsystemBase {

	private static final double[][] INTAKE_SPEEDS =
			new double[][] {
				{0.45, -0.75, 0.45, 0.45}, // Intake, Feeder, Conveyor, Positioning | Cube
				{0.4, -0.2, 0.4, 0.35} // Intake, Feeder, Conveyor, Positioning | Cone
			};

	private static final double[][] OUTTAKE_SPEEDS =
			new double[][] {
				{-0.15, 0.1, -0.3}, // Intake, Feeder, Conveyor | Cube
				{-0.15, 0.1, -0.3} // Intake, Feeder, Conveyor | Cone
			};

	private static final double[][] SHOOT_OUT_SPEEDS =
			new double[][] {
				{-0.25, 0.5, -0.5}, // Intake, Feeder, Conveyor | Cube
				{-0.25, 1.0, -0.5} // Intake, Feeder, Conveyor | Cone
			};

	private static final double[][] ARM_POSITIONS =
			new double[][] {
				{
					20000, 32500, 20000, 20000, 30000, 50000, 60000
				}, // Idling, Ground Pickup, Backstopping, Grabbing, Low, Mid, High | Cube
				{
					18000, 32500, 18000, 18000, 30000, 60000, 68500
				} // Idling, Ground Pickup, Backstopping, Grabbing, Low, Mid, High | Cone
			};

	private static final double[][] EXTENSION_POSITIONS =
			new double[][] {
				{
					-250, -11500, -250, -0, -500, -3750, -17875
				}, // Idling, Ground Pickup, Backstopping, Grabbing, Low, Mid, High | Cube
				{
					-3250, -11500, -3250, -1500, -500, -3500, -17875
				} // Idling, Ground Pickup, Backstopping, Grabbing, Low, Mid, High | Cone
			};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{
					0, 0, 0, -10000, -10000
				}, // Idling, Ground Pickup, Backstopping, Grabbing, Loose Grab | Cube
				{
					0, 0, 0, -20000, -17500
				} // Idling, Ground Pickup, Backstopping, Grabbing, Loose Grab | Cone
			};

	private static final double SCORE_TIME = 0.5; // seconds

	private static final double CONE_POSITION_TIME = 1000;
	private static final double CUBE_POSITION_TIME = 1000;

	private static final double CONE_GRABBING_MODE_TIME = 1;

	public enum IntakeState {
		Idling,
		Intaking,
		Positioning,
		Outtaking,
		ShootOuting
	}

	private enum ArmState {
		Idling,
		GroundPickup,
		Backstopping,
		Grabbing,
		Holding,
		Prepping,
		Scoring
	}

	public enum State {
		Idling,
		Intaking,
		GroundPickup,
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
	public final Arm m_arm;
	private final Gripper m_gripper;

	private IntakeState m_intakeState = IntakeState.Idling;
	private ArmState m_armState = ArmState.Idling;

	private State m_state = State.Idling;

	private Element m_element = Element.Cube;

	private LEDWrapper m_ledWrapper;

	private int[] m_targetNode = {0, 0};

	private final Timer m_scoreTimer = new Timer();
	private final Timer m_conePositionTimer = new Timer();
	private final Timer m_cubePositionTimer = new Timer();

	private final Timer m_coneGrabbingModeTimer = new Timer();

	private boolean m_autoScore = false;
	private boolean m_grab = false;

	private final SendableChooser<Double> m_offsetChooser;
	private final SendableChooser<Double> m_grabTightnessChooser;
	private final SendableChooser<Double> m_prepTightnessChooser;

	private final SendableChooser<Integer> m_armMotorChooser;

	public Superstructure(
			Intake intake,
			Feeder feeder,
			Conveyor conveyor,
			Arm arm,
			Gripper gripper,
			LEDWrapper ledWrapper) {
		m_intake = intake;
		m_feeder = feeder;
		m_conveyor = conveyor;
		m_arm = arm;
		m_gripper = gripper;

		m_ledWrapper = ledWrapper;

		m_offsetChooser = new SendableChooser<>();

		m_offsetChooser.addOption("UpUpUp", 8000.0);
		m_offsetChooser.addOption("UpUp", 4000.0);
		m_offsetChooser.addOption("Up", 2000.0);
		m_offsetChooser.setDefaultOption("Middle", 0.0);
		m_offsetChooser.addOption("Down", -2000.0);
		m_offsetChooser.addOption("DownDown", -4000.0);
		m_offsetChooser.addOption("DownDownDown", -8000.0);

		m_grabTightnessChooser = new SendableChooser<>();

		m_grabTightnessChooser.addOption("TightTight", -1000.0);
		m_grabTightnessChooser.addOption("Tight", -500.0);
		m_grabTightnessChooser.setDefaultOption("Middle", 0.0);
		m_grabTightnessChooser.addOption("Loose", 500.0);
		m_grabTightnessChooser.addOption("LooseLoose", 1000.0);

		m_prepTightnessChooser = new SendableChooser<>();

		m_prepTightnessChooser.addOption("TightTightTight", -750.0);
		m_prepTightnessChooser.addOption("TightTight", -500.0);
		m_prepTightnessChooser.addOption("Tight", -250.0);
		m_prepTightnessChooser.setDefaultOption("Middle", 0.0);
		m_prepTightnessChooser.addOption("Loose", 250.0);
		m_prepTightnessChooser.addOption("LooseLoose", 500.0);
		m_prepTightnessChooser.addOption("LooseLooseLoose", 750.0);

		SmartDashboard.putData("Arm Offset", m_offsetChooser);
		SmartDashboard.putData("Grab Tightness", m_grabTightnessChooser);
		SmartDashboard.putData("Prep Tightness", m_prepTightnessChooser);

		m_armMotorChooser = new SendableChooser<>();

		m_armMotorChooser.setDefaultOption("Left", 1);
		m_armMotorChooser.addOption("Right", 2);

		SmartDashboard.putData("Arm Motor", m_armMotorChooser);
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
			case GroundPickup:
				groundPickup();
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

		switch (m_targetNode[0]) {
			case 0:
				SmartDashboard.putString("Level", "Low");
				break;
			case 1:
				SmartDashboard.putString("Level", "Mid");
				break;
			case 2:
				SmartDashboard.putString("Level", "High");
				break;
		}

		if (m_element == Element.Cone) {
			m_ledWrapper.setColor(true, false);
		} else if (m_element == Element.Cube) {
			m_ledWrapper.setColor(false, true);
		}

		SmartDashboard.putBoolean("AutoScore", m_autoScore);
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

		m_arm.setArm(
				ARM_POSITIONS[m_element.index][0],
				EXTENSION_POSITIONS[m_element.index][0],
				m_armMotorChooser.getSelected());

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
	}

	public void intake() {
		if (m_intakeState != IntakeState.Intaking
				&& m_intakeState != IntakeState.Positioning
				&& m_intakeState != IntakeState.Outtaking) m_intakeState = IntakeState.Intaking;

		if (m_armState != ArmState.Backstopping) m_armState = ArmState.Backstopping;

		if (m_state != State.Intaking) m_state = State.Intaking;

		if (m_intakeState == IntakeState.Intaking) {
			if (true || m_conveyor.getDebouncedSensor(0)) {
				m_intakeState = IntakeState.Positioning;
				m_armState = ArmState.Grabbing;

				return;
			}

			m_intake.intake(INTAKE_SPEEDS[m_element.index][0]);
			m_feeder.feed(INTAKE_SPEEDS[m_element.index][1]);
			m_conveyor.convey(INTAKE_SPEEDS[m_element.index][2]);

			m_arm.setArm(
					ARM_POSITIONS[m_element.index][2],
					EXTENSION_POSITIONS[m_element.index][2],
					m_armMotorChooser.getSelected());

			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][2]);
		} else if (m_intakeState == IntakeState.Positioning) {
			if (m_element == Element.Cone) {
				if (!m_conePositionTimer.hasElapsed(0.01)) {
					m_conePositionTimer.start();
				}

				if (!m_coneGrabbingModeTimer.hasElapsed(0.01)) {
					m_coneGrabbingModeTimer.start();
				}

				if (m_conePositionTimer.hasElapsed(CONE_POSITION_TIME) || m_grab) {
					m_conePositionTimer.stop();
					m_conePositionTimer.reset();

					m_coneGrabbingModeTimer.stop();
					m_coneGrabbingModeTimer.reset();

					m_grab = false;

					hold();
					return;
				}

				if (m_coneGrabbingModeTimer.hasElapsed(CONE_GRABBING_MODE_TIME)) {
					m_arm.setArm(
							ARM_POSITIONS[m_element.index][3],
							EXTENSION_POSITIONS[m_element.index][3],
							m_armMotorChooser.getSelected());

					m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
				}
			} else if (m_element == Element.Cube) {
				if (!m_cubePositionTimer.hasElapsed(0.01)) {
					m_cubePositionTimer.start();
				}

				if (m_cubePositionTimer.hasElapsed(CUBE_POSITION_TIME) || m_grab) {
					m_cubePositionTimer.stop();
					m_cubePositionTimer.reset();

					m_grab = false;

					hold();
					return;
				}
			}

			//			m_intake.stop();
			//			m_feeder.stop();
			m_intake.intake(INTAKE_SPEEDS[m_element.index][0]);
			m_feeder.feed(INTAKE_SPEEDS[m_element.index][1]);
			m_conveyor.convey(INTAKE_SPEEDS[m_element.index][3]);

			m_arm.setArm(
					ARM_POSITIONS[m_element.index][2],
					EXTENSION_POSITIONS[m_element.index][2],
					m_armMotorChooser.getSelected());

			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
		}
	}

	public void groundPickup() {
		if (m_intakeState != IntakeState.Idling) m_intakeState = IntakeState.Idling;
		if (m_armState != ArmState.GroundPickup) m_armState = ArmState.GroundPickup;

		if (m_grab) {
			m_grab = false;

			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][3]);
		}

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		m_arm.setArm(
				ARM_POSITIONS[m_element.index][1],
				EXTENSION_POSITIONS[m_element.index][1],
				m_armMotorChooser.getSelected());

		if (m_state != State.GroundPickup) {
			m_state = State.GroundPickup;
			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
		}
	}

	public void outtake() {
		if (m_intakeState != IntakeState.Outtaking && m_intakeState != IntakeState.ShootOuting)
			m_intakeState = IntakeState.Outtaking;
		if (m_armState != ArmState.Idling) m_armState = ArmState.Idling;

		if (m_state != State.Outtaking) m_state = State.Outtaking;

		if (m_intakeState == IntakeState.Outtaking) {
			m_intake.outtake(OUTTAKE_SPEEDS[m_element.index][0]);
			m_feeder.outtake(OUTTAKE_SPEEDS[m_element.index][1]);
			m_conveyor.outtake(OUTTAKE_SPEEDS[m_element.index][2]);
		} else {
			m_intake.outtake(SHOOT_OUT_SPEEDS[m_element.index][0]);
			m_intake.outtake(SHOOT_OUT_SPEEDS[m_element.index][1]);
			m_intake.outtake(SHOOT_OUT_SPEEDS[m_element.index][2]);
		}

		m_arm.setArm(
				ARM_POSITIONS[m_element.index][0],
				EXTENSION_POSITIONS[m_element.index][0],
				m_armMotorChooser.getSelected());

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);
	}

	public void hold() {
		if (m_intakeState != IntakeState.Idling) m_intakeState = IntakeState.Idling;
		if (m_armState != ArmState.Holding) m_armState = ArmState.Holding;

		if (m_state != State.Holding) m_state = State.Holding;

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		m_arm.setArm(
				ARM_POSITIONS[m_element.index][3],
				EXTENSION_POSITIONS[m_element.index][3],
				m_armMotorChooser.getSelected());

		if (m_element == Element.Cone) {
			m_gripper.setClosedLoop(
					GRIPPER_POSITIONS[m_element.index][3] + m_grabTightnessChooser.getSelected());
		} else {
			m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][3]);
		}
	}

	public void prep() {
		if (m_intakeState != IntakeState.Idling) m_intakeState = IntakeState.Idling;
		if (m_armState != ArmState.Prepping) m_armState = ArmState.Prepping;

		if (m_state != State.Prepping) m_state = State.Prepping;

		m_intake.stop();
		m_feeder.stop();
		m_conveyor.stop();

		if (m_element == Element.Cone) {
			m_arm.setArm(
					ARM_POSITIONS[m_element.index][4 + m_targetNode[0]]
							+ m_offsetChooser.getSelected(),
					EXTENSION_POSITIONS[m_element.index][4 + m_targetNode[0]],
					m_armMotorChooser.getSelected());
		} else {
			m_arm.setArm(
					ARM_POSITIONS[m_element.index][4 + m_targetNode[0]],
					EXTENSION_POSITIONS[m_element.index][4 + m_targetNode[0]],
					m_armMotorChooser.getSelected());
		}

		if (!m_arm.isAtPosition()) {
			if (m_element == Element.Cone) {
				m_gripper.setClosedLoop(
						GRIPPER_POSITIONS[m_element.index][3]
								+ m_grabTightnessChooser.getSelected());
			} else {
				m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][3]);
			}
		} else {
			if (m_element == Element.Cone) {
				m_gripper.setClosedLoop(
						GRIPPER_POSITIONS[m_element.index][4]
								+ m_prepTightnessChooser.getSelected());
			} else {
				m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][4]);
			}
		}
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

		if (m_element == Element.Cone) {
			m_arm.setArm(
					ARM_POSITIONS[m_element.index][4 + m_targetNode[0]]
							+ m_offsetChooser.getSelected(),
					EXTENSION_POSITIONS[m_element.index][4 + m_targetNode[0]],
					m_armMotorChooser.getSelected());
		} else {
			m_arm.setArm(
					ARM_POSITIONS[m_element.index][4 + m_targetNode[0]],
					EXTENSION_POSITIONS[m_element.index][4 + m_targetNode[0]],
					m_armMotorChooser.getSelected());
		}

		m_gripper.setClosedLoop(GRIPPER_POSITIONS[m_element.index][0]);

		if (m_scoreTimer.hasElapsed(SCORE_TIME)) {
			m_scoreTimer.stop();
			m_scoreTimer.reset();

			idle();
		}
	}

	public void shootOut() {
		if (m_intakeState != IntakeState.ShootOuting) m_intakeState = IntakeState.ShootOuting;
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

	public void setAutoScore(boolean autoScore) {
		m_autoScore = autoScore;
	}

	public boolean getAutoScore() {
		return m_autoScore;
	}

	public State getState() {
		return m_state;
	}

	public IntakeState getIntakeState() {
		return m_intakeState;
	}

	public void grab() {
		m_grab = true;
	}
}
