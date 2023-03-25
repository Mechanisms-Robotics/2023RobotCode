package frc.robot.states;

import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public abstract class IntakeState implements State {
	protected final Intake m_intake;
	protected final Feeder m_feeder;
	protected final Conveyor m_conveyor;

	protected boolean m_initialized = false;

	public IntakeState(Intake intake, Feeder feeder, Conveyor conveyor) {
		m_intake = intake;
		m_feeder = feeder;
		m_conveyor = conveyor;
	}

	public void reset() {
		m_initialized = false;
	}
}
