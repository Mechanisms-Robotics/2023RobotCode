package frc.robot.states.intake;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.states.IntakeState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class Shlurping extends IntakeState {
	private static final double SHLURP_SPEED = 0.05;

	public Shlurping(Intake intake, Feeder feeder, Conveyor conveyor) {
		super(intake, feeder, conveyor);
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		m_intake.setBrakeMode(true);
	}

	@Override
	public void periodic() {
		if (!DriverStation.isEnabled()) return;

		if (!m_initialized) {
			init();
			m_initialized = true;
		}

		m_intake.setOpenLoop(SHLURP_SPEED);
		m_feeder.stop();
		m_conveyor.stop();
	}
}
