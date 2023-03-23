package frc.robot.states.intake;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.states.IntakeState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class Unjamming extends IntakeState {
	public Unjamming(Intake intake, Feeder feeder, Conveyor conveyor) {
		super(intake, feeder, conveyor);
	}

	@Override
	public void init() {
		if (!DriverStation.isEnabled()) return;

		m_intake.stop();
		m_feeder.setOpenLoop(1.0, -1.0);
		m_conveyor.stop();
	}

	@Override
	public void periodic() {
		if (!DriverStation.isEnabled()) return;
	}
}
