package frc.robot.states.intake;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.states.IntakeState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.Supplier;

public class Intaking extends IntakeState {
	private static final double[][] INTAKE_SPEEDS =
			new double[][] {
				{0.5, -0.75, 0.75}, // Intake, Feeder, Conveyor | Cube  0.75
				{0.5, -0.2, 0.5} // Intake, Feeder, Conveyor | Cone
			};

	private final Supplier<Element> m_elementSupplier;

	public Intaking(
			Intake intake, Feeder feeder, Conveyor conveyor, Supplier<Element> elementSupplier) {
		super(intake, feeder, conveyor);

		m_elementSupplier = elementSupplier;
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;
	}

	@Override
	public void periodic() {
		if (!DriverStation.isEnabled()) return;

		if (!m_initialized) {
			init();
			m_initialized = true;
		}

		m_intake.setOpenLoop(INTAKE_SPEEDS[m_elementSupplier.get().index][0]);
		m_feeder.setOpenLoop(INTAKE_SPEEDS[m_elementSupplier.get().index][1]);
		m_conveyor.setOpenLoop(INTAKE_SPEEDS[m_elementSupplier.get().index][2]);
	}
}
