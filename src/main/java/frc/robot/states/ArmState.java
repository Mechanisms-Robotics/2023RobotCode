package frc.robot.states;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public abstract class ArmState implements State {
	private static final double RETRACTED_POSITION = -500;

	protected final Arm m_arm;
	protected final Gripper m_gripper;

	protected final double m_desiredPosition;
	protected final double m_desiredExtension;

	protected final double m_openPosition;
	protected final double m_closedPosition;

	private enum ArmAction {
		Idling,
		Retracting,
		Pivoting,
		Extending
	}

	private enum GripperState {
		Opened,
		Closed
	}

	protected ArmAction m_currentAction = ArmAction.Idling;
	protected GripperState m_gripperState = GripperState.Opened;

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
		retract();
	}

	@Override
	public void periodic() {
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
		if (m_gripperState != GripperState.Opened) {
			m_gripperState = GripperState.Opened;
			m_gripper.setClosedLoop(m_openPosition);
		}
	}

	public void close() {
		if (m_gripperState != GripperState.Closed) {
			m_gripperState = GripperState.Closed;
			m_gripper.setClosedLoop(m_closedPosition);
		}
	}

	public boolean isOpen() {
		return m_gripperState == GripperState.Opened;
	}

	public boolean isClosed() {
		return m_gripperState == GripperState.Closed;
	}
}
