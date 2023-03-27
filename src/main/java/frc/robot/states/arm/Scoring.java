package frc.robot.states.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.states.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.Supplier;

public class Scoring extends ArmState {
	private static final double[][][] ARM_POSITIONS = {
		{
			{35000, -750},   // Low  | Cube
			{55000, -3750},  // Mid  | Cube
			{65000, -17875}, // High | Cube
		},
		{
			{35000, -1500},  // Low  | Cone
			{62500, -3750},  // Mid  | Cone
			{70000, -17500}, // High | Cone
		}
	};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{0, -14000, -14000}, // Open, Closed, Auto | Cube
				{0, -18500, -20000}  // Open, Closed, Auto | Cone
			};

	private static final double LOOSE_TIME = 5.0;

	private final Supplier<Element> m_elementSupplier;
	private final Supplier<Integer> m_levelSupplier;

	private Element m_prevElement;
	private int m_prevLevel;

	private final Timer m_looseTimer = new Timer();

	private boolean m_hasLoosened = false;

	public Scoring(
			Arm arm,
			Gripper gripper,
			Supplier<Element> elementSupplier,
			Supplier<Integer> levelSupplier) {
		super(
				arm,
				gripper,
				ARM_POSITIONS[elementSupplier.get().index][levelSupplier.get()][0],
				ARM_POSITIONS[elementSupplier.get().index][levelSupplier.get()][1],
				GRIPPER_POSITIONS,
				elementSupplier);

		m_elementSupplier = elementSupplier;
		m_levelSupplier = levelSupplier;

		m_prevElement = m_elementSupplier.get();
		m_prevLevel = m_levelSupplier.get();
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		super.init();
	}

	@Override
	public void periodic() {
		if (m_elementSupplier.get() != m_prevElement || m_levelSupplier.get() != m_prevLevel) {
			m_desiredPosition =
					ARM_POSITIONS[m_elementSupplier.get().index][m_levelSupplier.get()][0];
			m_desiredExtension =
					ARM_POSITIONS[m_elementSupplier.get().index][m_levelSupplier.get()][1];

			super.init();

			m_prevElement = m_elementSupplier.get();
			m_prevLevel = m_levelSupplier.get();
		} else {
			super.periodic();
		}

    if (m_arm.isExtended() && m_arm.extendAtPosition() && !m_hasLoosened) {
			close();

			m_looseTimer.start();

      System.out.println("STARTED LOOSE TIMER");

			m_hasLoosened = true;
		}

		if (m_looseTimer.hasElapsed(LOOSE_TIME)) {
			close(GRIPPER_POSITIONS[m_elementSupplier.get().index][2]);

      System.out.println("STOPPED LOOSE TIMER");

			m_looseTimer.stop();
			m_looseTimer.reset();
		}
	}
}
