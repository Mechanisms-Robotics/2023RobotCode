package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

	private static final double DEBOUNCE_TIME = 0.25; // seconds

	private static final TalonFXConfiguration CONVEYOR_MOTOR_CONFIG = new TalonFXConfiguration();

	static {
		final var conveyorCurrentLimit = new SupplyCurrentLimitConfiguration();
		conveyorCurrentLimit.currentLimit = 15; // Amps
		conveyorCurrentLimit.triggerThresholdCurrent = 18; // Amps
		conveyorCurrentLimit.triggerThresholdTime = 0.25; // sec
		conveyorCurrentLimit.enable = true;

		CONVEYOR_MOTOR_CONFIG.supplyCurrLimit = conveyorCurrentLimit;
	}

	private final WPI_TalonFX conveyorMotor = new WPI_TalonFX(40);

	public Conveyor() {
		conveyorMotor.configAllSettings(CONVEYOR_MOTOR_CONFIG);
		conveyorMotor.setInverted(true);
		conveyorMotor.setNeutralMode(NeutralMode.Coast);

		conveyorMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
	}

	public void setOpenLoop(double percentOutput) {
		conveyorMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void convey(double speed) {
		setOpenLoop(speed);
	}

	public void outtake(double speed) {
		setOpenLoop(speed);
	}

	public void stop() {
		setOpenLoop(0.0);
	}
}
