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
import frc.robot.states.intake.Unjamming;
import frc.robot.util.LEDWrapper;

public class Superstructure extends SubsystemBase {
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

	private final int[] m_targetNode = {0, 0};

	private final Timer m_scoreTimer = new Timer();

	private boolean m_autoScore = false;

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

		m_intakeState = new Idling(m_intake, m_feeder, m_conveyor);
		m_armState = new Stowed(m_arm, m_gripper, m_element);

		m_ledWrapper = ledWrapper;
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

		if (m_element == Element.Cone) {
			m_ledWrapper.setColor(true, false);
		} else if (m_element == Element.Cube) {
			m_ledWrapper.setColor(false, true);
		}

		SmartDashboard.putString("Intake State", m_intakeState.getClass().getSimpleName());
		SmartDashboard.putString("Arm State", m_armState.getClass().getSimpleName());

		SmartDashboard.putBoolean("Gripper", m_armState.isClosed());

		SmartDashboard.putBoolean("AutoScore", m_autoScore);
	}

	@Override
	public void simulationPeriodic() {
		super.simulationPeriodic();
		periodic();
	}

	public void idle() {
		if (m_intakeState.getClass() != Idling.class) {
			m_intakeState = new Idling(m_intake, m_feeder, m_conveyor);
			m_intakeState.init();
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, m_element);
			m_armState.init();

			open();
		} else {
			m_armState.periodic();
		}
	}

	public void intake() {
		if (m_intakeState.getClass() != Intaking.class) {
			m_intakeState = new Intaking(m_intake, m_feeder, m_conveyor, () -> m_element);
			m_intakeState.init();
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, m_element);
			m_armState.init();

			open();
		} else {
			m_armState.periodic();
		}
	}

	public void outtake() {
		if (m_intakeState.getClass() != Outtaking.class) {
			m_intakeState = new Outtaking(m_intake, m_feeder, m_conveyor, () -> m_element);
			m_intakeState.init();
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, m_element);
			m_armState.init();

			open();
		} else {
			m_armState.periodic();
		}
	}

	public void unjam() {
		if (m_intakeState.getClass() != Unjamming.class) {
			m_intakeState = new Unjamming(m_intake, m_feeder, m_conveyor);
			m_intakeState.init();
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Stowed.class) {
			m_armState = new Stowed(m_arm, m_gripper, m_element);
			m_armState.init();

			open();
		} else {
			m_armState.periodic();
		}
	}

	public void score() {
		if (m_intakeState.getClass() != Idling.class) {
			m_intakeState = new Idling(m_intake, m_feeder, m_conveyor);
			m_intakeState.init();
		} else {
			m_intakeState.periodic();
		}

		if (m_armState.getClass() != Scoring.class) {
			m_armState = new Scoring(m_arm, m_gripper, m_element, m_targetNode[0]);
			m_armState.init();

			close();
		} else {
			m_armState.periodic();
		}
	}

	public void open() {
		m_armState.open();
	}

	public void close() {
		m_armState.close();
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
			return m_arm.isAtPosition() && m_arm.extendAtPosition() && m_gripper.atPosition();
		} else {
			return true;
		}
	}
}
