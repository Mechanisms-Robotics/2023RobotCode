package frc.robot.states;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public abstract class ArmState implements State {
	private static final double RETRACTED_POSITION = -500;

	protected final Arm m_arm;
	protected final Gripper m_gripper;

	protected double m_desiredPosition;
	protected double m_desiredExtension;

	protected double m_openPosition;
	protected double m_closedPosition;

	protected boolean m_initialized = false;

	protected enum ArmAction {
		Idling,
		Retracting,
		Pivoting,
		Extending
	}

	protected ArmAction m_currentAction = ArmAction.Idling;

	public ArmState(
			Arm arm,
			Gripper gripper,
			double desiredPosition,
			double desiredExtension,
			double openPosition,
			double closedPosition) {
		m_arm = arm;
		m_gripper = gripper;

		m_desiredPosition = desiredPosition;
		m_desiredExtension = desiredExtension;

		m_openPosition = openPosition;
		m_closedPosition = closedPosition;
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		retract();
	}

	public void reset() {
		m_initialized = false;
	}

	@Override
	public void periodic() {
		if (!DriverStation.isEnabled()) return;

		if (!m_initialized) {
			init();
			m_initialized = true;
		}

		SmartDashboard.putString("Arm Action", m_currentAction.toString());

		switch (m_currentAction) {
			case Retracting:
				retract();
				break;
			case Pivoting:
				pivot();
				break;
			case Extending:
				extend();
				break;
			default:
				break;
		}
	}

	public void retract() {
		if (m_currentAction != ArmAction.Retracting) {
			m_currentAction = ArmAction.Retracting;

			m_arm.setExtensionClosedLoop(RETRACTED_POSITION);
		} else if (m_arm.extendAtPosition()) {
			pivot();
		}
	}

	public void pivot() {
		if (m_currentAction != ArmAction.Pivoting) {
			m_currentAction = ArmAction.Pivoting;

			m_arm.setClosedLoop(m_desiredPosition);
		} else if (m_arm.isAtPosition()) {
			extend();
		}

		SmartDashboard.putBoolean("Arm At Position", m_arm.isAtPosition());
	}

	public void extend() {
		if (m_currentAction != ArmAction.Extending) {
			m_currentAction = ArmAction.Extending;

			m_arm.setExtensionClosedLoop(m_desiredExtension);
		} else if (m_arm.extendAtPosition()) {
			m_currentAction = ArmAction.Idling;
		}
	}

	public void open() {
		m_gripper.setClosedLoop(m_openPosition);
	}

	public void close() {
		m_gripper.setClosedLoop(m_closedPosition);
	}

	public boolean isOpen() {
		return m_gripper.isOpen();
	}

	public boolean isClosed() {
		return m_gripper.isClosed();
	}
}
