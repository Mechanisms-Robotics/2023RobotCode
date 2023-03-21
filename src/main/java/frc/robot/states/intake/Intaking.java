package frc.robot.states.intake;

import frc.robot.states.IntakeState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.Supplier;

public class Intaking extends IntakeState {
	private static final double[][] INTAKE_SPEEDS =
			new double[][] {
				{0.45, -0.75, 0.45}, // Intake, Feeder, Conveyor | Cube
				{0.4, -0.2, 0.4} // Intake, Feeder, Conveyor | Cone
			};

	private final Supplier<Element> m_elementSupplier;

	public Intaking(
			Intake intake, Feeder feeder, Conveyor conveyor, Supplier<Element> elementSupplier) {
		super(intake, feeder, conveyor);

		m_elementSupplier = elementSupplier;
	}

	@Override
	public void init() {}

	@Override
	public void periodic() {
		m_intake.setOpenLoop(INTAKE_SPEEDS[m_elementSupplier.get().index][0]);
		m_feeder.setOpenLoop(INTAKE_SPEEDS[m_elementSupplier.get().index][1]);
		m_conveyor.setOpenLoop(INTAKE_SPEEDS[m_elementSupplier.get().index][2]);
	}
}
