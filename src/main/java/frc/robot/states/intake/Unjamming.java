package frc.robot.states.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.states.IntakeState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class Unjamming extends IntakeState {
	private static final double ROTATE_TIME = 1.0; // seconds

	private final Timer m_rotateTimer = new Timer();
	private boolean m_rotateDirection = false;

	public Unjamming(Intake intake, Feeder feeder, Conveyor conveyor) {
		super(intake, feeder, conveyor);
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		m_intake.setOpenLoop(-0.3);
		m_conveyor.setOpenLoop(-0.75);

		m_rotateTimer.start();
	}

	@Override
	public void periodic() {
		if (!DriverStation.isEnabled()) return;

		if (!m_initialized) {
			init();
			m_initialized = true;
		}

		if (m_rotateTimer.hasElapsed(ROTATE_TIME)) {
			m_rotateDirection = !m_rotateDirection;
			m_rotateTimer.restart();
		}

		if (m_rotateDirection) {
			m_feeder.setOpenLoop(1.0, -1.0);
		} else {
			m_feeder.setOpenLoop(-1.0, 1.0);
		}
	}
}
