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
			{12000, -750}, // Low  | Cube
			{23000, -3750}, // Mid  | Cube
			{32500, -18000}, // High | Cube | 30000
		},
		{
			{12000, -1500}, // Low  | Cone
			{27000, -4000}, // Mid  | Cone
			{35000, -18000}, // High | Cone | 33000
			{30000, -18000}, // Place | Cone
		}
	};

	private static final double[][] GRIPPER_POSITIONS =
			new double[][] {
				{0, -17500, -17500}, // Open, Closed, Auto | Cube
				{0, -27500, -27500} // Open, Closed, Auto | Cone
			};

	private static final double OPEN_INCREMENT = 7.5;
	private static final double LOOSE_TIME = 3.0;

	private final Supplier<Element> m_elementSupplier;
	private final Supplier<Integer> m_levelSupplier;
	private Supplier<Double> m_loosenSupplier;
	private Supplier<Double> m_jogSupplier;

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
				elementSupplier,
				() -> levelSupplier.get() == 3);

		m_elementSupplier = elementSupplier;
		m_levelSupplier = levelSupplier;

		m_prevElement = m_elementSupplier.get();
		m_prevLevel = m_levelSupplier.get();

		m_loosenSupplier = null;
		m_jogSupplier = null;
	}

	public Scoring(
			Arm arm,
			Gripper gripper,
			Supplier<Element> elementSupplier,
			Supplier<Integer> levelSupplier,
			Supplier<Double> loosenSupplier,
			Supplier<Double> jogSupplier) {
		this(arm, gripper, elementSupplier, levelSupplier);

		m_loosenSupplier = loosenSupplier;
		m_jogSupplier = jogSupplier;
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

		if (!m_autoReleasing) {
			if (m_jogSupplier != null && Math.abs(m_jogSupplier.get()) >= 0.1) {
				m_arm.jogArm(m_jogSupplier.get(), m_elementSupplier);
				m_arm.setClosedLoop(m_desiredPosition);
			}

			if (m_loosenSupplier == null) {
				if (m_arm.isExtended() && m_arm.extendAtPosition() && !m_hasLoosened) {
					close();

					m_looseTimer.start();

					m_hasLoosened = true;
				}

				if (m_looseTimer.hasElapsed(LOOSE_TIME)) {
					close(GRIPPER_POSITIONS[m_elementSupplier.get().index][2]);

					m_looseTimer.stop();
					m_looseTimer.reset();
				}
			} else if (m_elementSupplier.get()
					!= Element.Cube) { // TODO: Reimplement m_desiredGripper
				double desiredPosition =
						GRIPPER_POSITIONS[1][2] + (2500 * Math.pow(m_loosenSupplier.get(), 2));
				close(desiredPosition);
			}
		} else {
			autoRelease(GRIPPER_POSITIONS[1][2], OPEN_INCREMENT);
		}
	}
}
