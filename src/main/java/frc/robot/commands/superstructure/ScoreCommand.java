package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;
import java.util.function.BooleanSupplier;

public class ScoreCommand extends CommandBase {
	private final Superstructure m_superstructure;
	private final BooleanSupplier m_interrupt;

	public ScoreCommand(Superstructure superstructure, BooleanSupplier interrupt) {
		m_superstructure = superstructure;
		m_interrupt = interrupt;

		addRequirements(superstructure);
	}

	@Override
	public void initialize() {
		m_superstructure.prep();
	}

	@Override
	public boolean isFinished() {
		return m_interrupt.getAsBoolean();
	}

	@Override
	public void end(boolean interrupted) {
		m_superstructure.score();
	}
}
