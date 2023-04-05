package frc.robot.states.arm;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.states.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.Supplier;

public class Stowed extends ArmState {
	private static final double[][] ARM_POSITIONS = {
		{26000, -250}, // Pivot, Extension | Cube
		{17000, -500} // Pivot, Extension | Cone
	};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{0, -14000, -14000}, // Open, Closed, Auto | Cube
				{0, -20000, -20000} // Open, Closed, Auto | Cone
			};

	private final Supplier<Element> m_elementSupplier;
	private Element m_prevElement = null;

	public Stowed(Arm arm, Gripper gripper, Supplier<Element> elementSupplier) {
		super(
				arm,
				gripper,
				ARM_POSITIONS[elementSupplier.get().index][0],
				ARM_POSITIONS[elementSupplier.get().index][1],
				GRIPPER_POSITIONS,
				elementSupplier);

		m_elementSupplier = elementSupplier;
		m_prevElement = m_elementSupplier.get();
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		open();
		super.init();
	}

	@Override
	public void periodic() {
		if (m_elementSupplier.get() != m_prevElement) {
			m_desiredPosition = ARM_POSITIONS[m_elementSupplier.get().index][0];
			m_desiredExtension = ARM_POSITIONS[m_elementSupplier.get().index][1];

			super.init();

			m_prevElement = m_elementSupplier.get();
		} else {
			super.periodic();
		}
	}
}
