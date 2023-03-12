package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Element;
import java.util.function.BooleanSupplier;

public class ScoreCommand extends CommandBase {
	private static final double HOLD_TIME = 0.25; // seconds
	private static final double SCORE_TIME = 3.0; // seconds

	private final Superstructure m_superstructure;
	private final BooleanSupplier m_interrupt;
	private final int m_level;
	private final Element m_element;

	private final Timer m_holdTimer = new Timer();
	private final Timer m_scoreTimer = new Timer();

	public ScoreCommand(Superstructure superstructure) {
		m_superstructure = superstructure;
		m_interrupt = null;
		m_level = -1;
		m_element = null;

		addRequirements(superstructure);
	}

	public ScoreCommand(Superstructure superstructure, BooleanSupplier interrupt) {
		m_superstructure = superstructure;
		m_interrupt = interrupt;
		m_level = -1;
		m_element = null;

		addRequirements(superstructure);
	}

	public ScoreCommand(Superstructure superstructure, int level, Element element) {
		m_superstructure = superstructure;
		m_level = level;
		m_element = element;
		m_interrupt = null;

		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		m_superstructure.hold();

		m_holdTimer.start();

		if (m_interrupt == null) {
			m_scoreTimer.start();
		}

		if (m_level != -1) {
			m_superstructure.setNode(m_level, 0);
		}

		if (m_element != null) {
			m_superstructure.setElement(m_element);
		}
	}

	@Override
	public void execute() {
		if (m_holdTimer.hasElapsed(HOLD_TIME)) {
			m_superstructure.prep();

			m_holdTimer.stop();
			m_holdTimer.reset();
		}
	}

	@Override
	public boolean isFinished() {
    if (m_interrupt != null) {
      return m_interrupt.getAsBoolean();
		} else {
			return m_scoreTimer.hasElapsed(SCORE_TIME);
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_superstructure.score();
	}
}
