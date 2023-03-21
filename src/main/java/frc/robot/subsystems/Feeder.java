package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code that controls the feeder functionality */
public class Feeder extends SubsystemBase {

	// Feeder motors
	private final WPI_TalonFX rightFeederMotor = new WPI_TalonFX(30);
	private final WPI_TalonFX leftFeederMotor = new WPI_TalonFX(31);

	// Feeder motor configuration
	private static final TalonFXConfiguration FEEDER_MOTOR_CONFIGURATION =
			new TalonFXConfiguration();

	// Configure the feeder current limit
	static {
		// Instantiate a new SupplyCurrentLimitConfiguration
		final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();

		// Configure the settings of the SupplyCurrentLimitConfiguration
		feederCurrentLimit.currentLimit = 10; // 15 amps
		feederCurrentLimit.triggerThresholdCurrent = 18; // amps
		feederCurrentLimit.triggerThresholdTime = 0.25; // sec
		feederCurrentLimit.enable = true;

		// Set the FEEDER_MOTOR_CONFIGURATION current limit and disable reverse and forward limits
		FEEDER_MOTOR_CONFIGURATION.supplyCurrLimit = feederCurrentLimit;
		FEEDER_MOTOR_CONFIGURATION.reverseSoftLimitEnable = false;
		FEEDER_MOTOR_CONFIGURATION.forwardSoftLimitEnable = false;
		FEEDER_MOTOR_CONFIGURATION.voltageCompSaturation = 10; // v
	}

	/** Constructor for the Feeder class */
	public Feeder() {
		rightFeederMotor.configFactoryDefault();
		leftFeederMotor.configFactoryDefault();

		rightFeederMotor.setInverted(TalonFXInvertType.Clockwise);
		rightFeederMotor.setNeutralMode(NeutralMode.Coast);
		rightFeederMotor.enableVoltageCompensation(true);

		leftFeederMotor.setInverted(TalonFXInvertType.CounterClockwise);
		leftFeederMotor.setNeutralMode(NeutralMode.Coast);
		leftFeederMotor.follow(rightFeederMotor);
		rightFeederMotor.setInverted(true);

		leftFeederMotor.setInverted(InvertType.OpposeMaster);

		rightFeederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		rightFeederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
	}

	public void setOpenLoop(double percent) {
		leftFeederMotor.set(ControlMode.PercentOutput, percent);
		rightFeederMotor.set(ControlMode.PercentOutput, percent);
	}

	/** Stops the feeder */
	public void stop() {
		leftFeederMotor.set(ControlMode.PercentOutput, 0.0);
		rightFeederMotor.set(ControlMode.PercentOutput, 0.0);
	}
}
