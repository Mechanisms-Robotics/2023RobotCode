package frc.robot.states;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.Supplier;

public abstract class ArmState implements State {
	private static final double RETRACTED_POSITION = -500;

	protected final Arm m_arm;
	protected final Gripper m_gripper;

	protected double m_desiredPosition;
	protected double m_desiredExtension;

	protected double[][] m_positions;
	protected Supplier<Element> m_ElementSupplier;
	
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
			double[][] positions,
			Supplier<Element> elementSupplier) {
		m_arm = arm;
		m_gripper = gripper;

		m_desiredPosition = desiredPosition;
		m_desiredExtension = desiredExtension;

		m_positions = positions;
		m_ElementSupplier = elementSupplier;
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
		m_gripper.setOpenLoop(0.15);
		m_gripper.setDesiredPosition(0.0);
	}

	public void close() {
		if (DriverStation.isAutonomousEnabled()) {
      System.out.println("AUTO POSITION: " + m_positions[m_ElementSupplier.get().index][2]);
			m_gripper.setClosedLoop(m_positions[m_ElementSupplier.get().index][2]);
		} else {
			m_gripper.setClosedLoop(m_positions[m_ElementSupplier.get().index][1]);
		}
	}

	public void close(double position) {
		m_gripper.setClosedLoop(position);
	}

	public boolean isOpen() {
		return m_gripper.isOpen();
	}

	public boolean isClosed() {
		return m_gripper.isClosed();
	}
}
