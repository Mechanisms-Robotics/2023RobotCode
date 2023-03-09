package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;

public class CubeFeedCommand extends CommandBase {
	private static final double IN_TIME = 2.0;
	private static final double OUT_TIME = 0.0;
	private static final double ROTATE_TIME = 0.0;
	private static final double POSITIONING_TIME = 1.375;

	private final Feeder m_feeder;
	private final Conveyor m_conveyor;

	private final Timer m_inTimer = new Timer();
	private final Timer m_outTimer = new Timer();
	private final Timer m_rotateTimer = new Timer();
	private final Timer m_positioningTimer = new Timer();

	private boolean m_sensorTriggered;

	private enum FeederState {
		Feeding,
		Unjamming,
		Rotating,
		Positioning,
		Stopped
	}

	private FeederState m_feederState = FeederState.Feeding;

	public CubeFeedCommand(Feeder feeder, Conveyor conveyor) {
		m_feeder = feeder;
		m_conveyor = conveyor;
		addRequirements(feeder, conveyor);
	}

	@Override
	public void initialize() {
		m_feeder.feed(0.0);
		m_conveyor.convey(0.0);

		m_inTimer.start();

		m_feederState = FeederState.Feeding;

		m_sensorTriggered = false;
	}

	@Override
	public void execute() {
		if (!m_conveyor.conveyorSensor.get() && !m_sensorTriggered) {
			m_sensorTriggered = true;

			m_feeder.stop();
			m_conveyor.position();

			m_positioningTimer.start();

			m_feederState = FeederState.Positioning;
		}

		switch (m_feederState) {
			case Stopped:
				m_inTimer.stop();
				m_inTimer.reset();

				m_outTimer.stop();
				m_outTimer.reset();

				m_rotateTimer.stop();
				m_rotateTimer.reset();

				m_positioningTimer.stop();
				m_positioningTimer.reset();

				break;
			case Feeding:
				if (m_inTimer.hasElapsed(IN_TIME)) {
					m_inTimer.stop();
					m_inTimer.reset();

					//          m_feeder.unjam();
					//          m_conveyor.unjam();
					m_feederState = FeederState.Unjamming;

					m_outTimer.start();
				}

				break;
			case Unjamming:
				if (m_outTimer.hasElapsed(OUT_TIME)) {
					m_outTimer.stop();
					m_outTimer.reset();

					//          m_feeder.rotate();
					//          m_conveyor.stop();
					m_feederState = FeederState.Rotating;

					m_rotateTimer.start();
				}

				break;
			case Rotating:
				if (m_rotateTimer.hasElapsed(ROTATE_TIME)) {
					m_rotateTimer.stop();
					m_rotateTimer.reset();

					m_feeder.feed(0);
					m_conveyor.convey(0);
					m_feederState = FeederState.Feeding;

					m_inTimer.start();
				}

				break;
			case Positioning:
				if (m_positioningTimer.hasElapsed(POSITIONING_TIME)) {
					m_conveyor.stop();

					m_positioningTimer.stop();
					m_positioningTimer.reset();

					m_feederState = FeederState.Stopped;
				}
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_feeder.stop();
		m_conveyor.stop();

		m_feederState = FeederState.Stopped;

		m_inTimer.stop();
		m_inTimer.reset();

		m_outTimer.stop();
		m_outTimer.reset();

		m_rotateTimer.stop();
		m_rotateTimer.reset();
	}
}
