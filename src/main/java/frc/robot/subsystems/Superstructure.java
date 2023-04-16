package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ArmState;
import frc.robot.states.IntakeState;
import frc.robot.states.arm.Scoring;
import frc.robot.states.arm.Stowed;
import frc.robot.states.intake.Idling;
import frc.robot.states.intake.Intaking;
import frc.robot.states.intake.Outtaking;
import frc.robot.states.intake.Shlurping;
import frc.robot.states.intake.Shooting;
import frc.robot.states.intake.Unjamming;
import frc.robot.util.LEDWrapper;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
	private static final double SCORE_BLINK_TIME = 1.0; // seconds

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

	private IntakeState m_intakeState;
	private ArmState m_armState;

	private Element m_element = Element.Cube;

	private final LEDWrapper m_ledWrapper;

	private final Supplier<Boolean> m_reverseSupplier;
	private final Supplier<Double> m_loosenSupplier;
	private final Supplier<Double> m_jogSupplier;

	private final int[] m_targetNode = {0, 0};

	private boolean m_scoreBlinking = false;
	private final Timer m_scoreBlinkTimer = new Timer();

	private boolean m_autoScore = false;

	private enum SuperstructureState {
		Idling,
		Intaking,
		Shlurping,
		Outtaking,
		Unjamming,
		Shooting,
		Scoring
	}

	private SuperstructureState m_superstructureState = SuperstructureState.Idling;

	public Superstructure(
			Intake intake,
			Feeder feeder,
			Conveyor conveyor,
			Arm arm,
			Gripper gripper,
			LEDWrapper ledWrapper,
			Supplier<Boolean> reverseSupplier,
			Supplier<Double> loosenSupplier,
			Supplier<Double> jogSupplier) {
		m_intake = intake;
		m_feeder = feeder;
		m_conveyor = conveyor;
		m_arm = arm;
		m_gripper = gripper;

		m_intakeState = new Idling(m_intake, m_feeder, m_conveyor);
		m_armState = new Stowed(m_arm, m_gripper, () -> m_element);

		m_ledWrapper = ledWrapper;

		m_reverseSupplier = reverseSupplier;
		m_loosenSupplier = loosenSupplier;
		m_jogSupplier = jogSupplier;
	}

	@Override
	public void periodic() {
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

		if (m_intakeState.getClass() != Outtaking.class
				&& m_intakeState.getClass() != Unjamming.class
				&& m_intakeState.getClass() != Shooting.class) {
			if (m_armState.getClass() != Scoring.class) {
				if (m_scoreBlinkTimer.hasElapsed(SCORE_BLINK_TIME)) {
					m_ledWrapper.setBlinking(false);

					m_scoreBlinkTimer.stop();
					m_scoreBlinkTimer.reset();

					m_scoreBlinking = false;
				}

				if (!m_scoreBlinking) {
          if (m_intakeState.getClass() != Shlurping.class) {
            if (m_element == Element.Cone) {
              m_ledWrapper.setColor(new boolean[] {true, true, false});
            } else if (m_element == Element.Cube) {
              m_ledWrapper.setColor(new boolean[] {true, false, true});
            }
					} else {
						m_ledWrapper.setColor(new boolean[] {true, true, true});
					}
				}
			} else {
				m_ledWrapper.setColor(new boolean[] {false, false, true});
			}
		} else {
			if (m_intakeState.getClass() == Outtaking.class
					|| m_intakeState.getClass() == Unjamming.class) {
				m_ledWrapper.setColor(new boolean[] {true, false, false});
			} else {
				m_ledWrapper.setColor(new boolean[] {true, true, true});
			}
		}

		switch (m_superstructureState) {
			case Idling:
				idle();
				break;
			case Intaking:
				intake();
				break;
			case Shlurping:
				shlurp();
				break;
			case Outtaking:
				outtake();
				break;
			case Unjamming:
				unjam();
				break;
			case Shooting:
				shoot();
				break;
			case Scoring:
				score();
				break;
		}

		SmartDashboard.putString("Intake State", m_intakeState.getClass().getSimpleName());
		SmartDashboard.putString("Arm State", m_armState.getClass().getSimpleName());
		SmartDashboard.putBoolean("Gripper Closed", m_armState.isClosed());

		SmartDashboard.putBoolean("AutoScore", m_autoScore);
	}

	@Override
	public void simulationPeriodic() {
		super.simulationPeriodic();
		periodic();
	}

	public void idle() {
		m_superstructureState = SuperstructureState.Idling;

		if (m_intakeState.getClass() != Idling.class) {
			m_intakeState = new Idling(m_intake, m_feeder, m_conveyor);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, () -> m_element);
			m_armState.setDeisredGripper(false);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(false);
	}

	public void intake() {
		m_superstructureState = SuperstructureState.Intaking;

		if (m_intakeState.getClass() != Intaking.class) {
			m_intakeState =
					new Intaking(
							m_intake, m_feeder, m_conveyor, () -> m_element, m_reverseSupplier);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, () -> m_element);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(true);
	}

	public void shlurp() {
		m_superstructureState = SuperstructureState.Shlurping;

		if (m_intakeState.getClass() != Shlurping.class) {
			m_intakeState = new Shlurping(m_intake, m_feeder, m_conveyor);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, () -> m_element);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(true);
	}

	public void outtake() {
		m_superstructureState = SuperstructureState.Outtaking;

		if (m_intakeState.getClass() != Outtaking.class) {
			m_intakeState = new Outtaking(m_intake, m_feeder, m_conveyor, () -> m_element);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, () -> m_element);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(true);
	}

	public void unjam() {
		m_superstructureState = SuperstructureState.Unjamming;

		if (m_intakeState.getClass() != Unjamming.class) {
			m_intakeState = new Unjamming(m_intake, m_feeder, m_conveyor);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, () -> m_element);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(true);
	}

	public void shoot() {
		m_superstructureState = SuperstructureState.Shooting;

		if (m_intakeState.getClass() != Shooting.class) {
			m_intakeState = new Shooting(m_intake, m_feeder, m_conveyor, () -> m_element);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, () -> m_element);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(true);
	}

	public void score() {
		m_superstructureState = SuperstructureState.Scoring;

		if (m_intakeState.getClass() != Idling.class) {
			m_intakeState = new Idling(m_intake, m_feeder, m_conveyor);
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Scoring.class) {
			if (m_element == Element.Cube && m_targetNode[0] == 3) {
				m_targetNode[0] = 2;
			}

			m_armState =
					new Scoring(
							m_arm,
							m_gripper,
							() -> m_element,
							() -> m_targetNode[0],
							m_loosenSupplier,
							m_jogSupplier);
			m_armState.setDeisredGripper(true);
		} else {
			m_armState.periodic();
		}

		m_ledWrapper.setBlinking(false);
	}

	public void open() {
		m_armState.setDeisredGripper(false);

		if (m_armState.getClass() == Scoring.class) {
			m_ledWrapper.setColor(new boolean[] {false, true, false});
			m_ledWrapper.setBlinking(true);

			m_scoreBlinking = true;
			m_scoreBlinkTimer.start();
		}

		m_armState.open();
	}

	public void close() {
		m_armState.setDeisredGripper(true);
		m_armState.close();
	}

	public void autoRelease() {
		m_armState.autoRelease();
	}

	public void reset() {
		m_intakeState.reset();
		m_armState.reset();
	}

	public void setElement(Element element) {
		m_element = element;
	}

	public void setNode(int row, int col) {
		m_targetNode[0] = row;
		m_targetNode[1] = col;
	}

	public void setAutoScore(boolean autoScore) {
		m_autoScore = autoScore;
	}

	public IntakeState getIntakeState() {
		return m_intakeState;
	}

	public ArmState getArmState() {
		return m_armState;
	}

	public boolean getAutoScore() {
		return m_autoScore;
	}

	public boolean atPosition() {
		if (RobotBase.isReal()) {
			System.out.println("ARM AT POSITION: " + m_arm.isAtPosition());
			System.out.println("EXTENSION AT POSITION: " + m_arm.extendAtPosition());
			System.out.println("GRIPPER AT POSITION: " + m_gripper.atPosition());
			return m_arm.isAtPosition() && m_arm.extendAtPosition() && m_gripper.atPosition();
		} else {
			return true;
		}
	}
}
