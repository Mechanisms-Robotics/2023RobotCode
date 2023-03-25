package frc.robot.states.arm;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.states.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.Supplier;

public class Scoring extends ArmState {
	private static final double[][][] ARM_POSITIONS = {
		{
			{35000, -750}, //   Low  | Cube
			{50000, -3750}, //  Mid  | Cube
			{60000, -17875}, // High | Cube
		},
		{
			{35000, -750}, //   Low  | Cone
			{62500, -3500}, //  Mid  | Cone
			{68750, -16500}, // High | Cone
		}
	};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{0, -12500}, // Open, Closed | Cube
				{0, -17500} // Open, Closed | Cone
			};

	private final Supplier<Element> m_elementSupplier;
	private final Supplier<Integer> m_levelSupplier;

	private Element m_prevElement;
	private int m_prevLevel;

	public Scoring(Arm arm, Gripper gripper, Supplier<Element> elementSupplier, Supplier<Integer> levelSupplier) {
		super(
				arm,
				gripper,
				ARM_POSITIONS[elementSupplier.get().index][levelSupplier.get()][0],
				ARM_POSITIONS[elementSupplier.get().index][levelSupplier.get()][1],
				GRIPPER_POSITIONS[elementSupplier.get().index][0],
				GRIPPER_POSITIONS[elementSupplier.get().index][1]);

		m_elementSupplier = elementSupplier;
		m_levelSupplier = levelSupplier;

		m_prevElement = m_elementSupplier.get();
		m_prevLevel = m_levelSupplier.get();
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		close();
		super.init();
	}

	@Override
	public void periodic() {
		if (m_elementSupplier.get() != m_prevElement || m_levelSupplier.get() != m_prevLevel) {
			m_desiredPosition = ARM_POSITIONS[m_elementSupplier.get().index][m_levelSupplier.get()][0];
			m_desiredExtension = ARM_POSITIONS[m_elementSupplier.get().index][m_levelSupplier.get()][1];

			m_openPosition = GRIPPER_POSITIONS[m_elementSupplier.get().index][0];
			m_closedPosition = GRIPPER_POSITIONS[m_elementSupplier.get().index][1];

			super.init();

			m_prevElement = m_elementSupplier.get();
			m_prevLevel = m_levelSupplier.get();
		} else {
			super.periodic();
		}

		if (m_currentAction == ArmAction.Extending) {
			close();
		}
	}
}
