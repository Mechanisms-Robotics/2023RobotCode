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

	private static final double DEBOUNCE_TIME = 0.25; // seconds

	private static final TalonFXConfiguration CONVEYOR_MOTOR_CONFIG = new TalonFXConfiguration();

	public final DigitalInput frontSensor = new DigitalInput(0);
	public final DigitalInput backSensor = new DigitalInput(1);

	private final boolean[] sensorValues = {true, true};
	private final boolean[] isDebouncing = {false, false};
	private final Timer[] debounceTimers = {new Timer(), new Timer()};

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
		SmartDashboard.putBoolean("Front Sensor", getDebouncedSensor(0));
		SmartDashboard.putBoolean("Back Sensor", getDebouncedSensor(1));
	}

	private void setOpenLoop(double percentOutput) {
		conveyorMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void convey(double speed) {
		setOpenLoop(speed);
	}

	public void outtake(double speed) {
		setOpenLoop(speed);
	}

	public boolean getDebouncedSensor(int id) {
		boolean rawSensorValue = id == 0 ? frontSensor.get() : backSensor.get();

		if (rawSensorValue != sensorValues[id]) {
			if (!isDebouncing[id]) {
				debounceTimers[id].start();

				isDebouncing[id] = true;
			} else {
				if (debounceTimers[id].hasElapsed(DEBOUNCE_TIME)) {
					sensorValues[id] = rawSensorValue;
				}
			}
		} else {
			debounceTimers[id].stop();
			debounceTimers[id].reset();

			isDebouncing[id] = false;
		}

		return !sensorValues[id];
	}

	public void stop() {
		setOpenLoop(0.0);
	}
}
