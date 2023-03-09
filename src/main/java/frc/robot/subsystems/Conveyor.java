package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

	private static final double CONVEYOR_CONVEY_SPEED = 0.45;
	private static final double CONVEYOR_POSITIONING_SPEED = 0.15;
	private static final double CONVEYOR_UNJAM_SPEED = -0.3;

	private static final double DEBOUNCE_TIME = 0.25; // seconds

	private static final TalonFXConfiguration CONVEYOR_MOTOR_CONFIG = new TalonFXConfiguration();

	public final DigitalInput conveyorSensor = new DigitalInput(0);

	private boolean sensorValue = true;
	private boolean isDebouncing = false;
	private Timer debounceTimer = new Timer();

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

		conveyorMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Conveyor Sensor", getDebouncedSensor());
	}

	private void setOpenLoop(double percentOutput) {
		conveyorMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void convey(double speed) {
		setOpenLoop(speed);
	}

	public void unjam() {
		setOpenLoop(CONVEYOR_UNJAM_SPEED);
	}

	public void position() {
		setOpenLoop(CONVEYOR_POSITIONING_SPEED);
	}

	public boolean getDebouncedSensor() {
		if (conveyorSensor.get() != sensorValue) {
			if (!isDebouncing) {
				debounceTimer.start();

				isDebouncing = true;
			} else {
				if (debounceTimer.hasElapsed(DEBOUNCE_TIME)) {
					sensorValue = conveyorSensor.get();
				}
			}
		} else {
			debounceTimer.stop();
			debounceTimer.reset();

			isDebouncing = false;
		}

		return sensorValue;
	}

	public void stop() {
		setOpenLoop(0.0);
	}
}
