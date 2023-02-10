package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code that controls the feeder functionality */
public class Feeder extends SubsystemBase {
	// Feeder speeds
	private static final double FEEDER_INTAKE_SPEED = 0.3;

	// Feeder motor
	private final WPI_TalonFX feederMotor = new WPI_TalonFX(30);
	private final WPI_TalonFX feederMotorFollower = new WPI_TalonFX(32);
	// Feeder motor configuration
	private static final TalonFXConfiguration FEEDER_MOTOR_CONFIGURATION =
			new TalonFXConfiguration();

	// Configure the feeder current limit
	static {
		// Instantiate a new SupplyCurrentLimitConfiguration
		final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();

		// Configure the settings of the SupplyCurrentLimitConfiguration
		feederCurrentLimit.currentLimit = 15; // amps
		feederCurrentLimit.triggerThresholdCurrent = 18; // amps
		feederCurrentLimit.triggerThresholdTime = 0.25; // sec
		feederCurrentLimit.enable = true;

		// Set the FEEDER_MOTOR_CONFIGURATION current limit and disable reverse and forward limits
		FEEDER_MOTOR_CONFIGURATION.supplyCurrLimit = feederCurrentLimit;
		FEEDER_MOTOR_CONFIGURATION.reverseSoftLimitEnable = false;
		FEEDER_MOTOR_CONFIGURATION.forwardSoftLimitEnable = false;
	}

	/** Constructor for the Feeder class */
	public Feeder() {
		// Configure feeder motor
		feederMotor.setInverted(TalonFXInvertType.Clockwise);
		feederMotor.setNeutralMode(NeutralMode.Coast);

		feederMotorFollower.setInverted(TalonFXInvertType.CounterClockwise);
		feederMotorFollower.setNeutralMode(NeutralMode.Coast);
		feederMotorFollower.follow(feederMotor);

		// CAN bus utilization optimization
		feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
		feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
	}

	/** Runs the feeder differently depending on which proximity sensors are triggered */
	public void intake() {
		feederMotor.set(ControlMode.PercentOutput, FEEDER_INTAKE_SPEED);
	}

	/** Stops the feeder */
	public void stop() {
		// Set the feeder motor to run at 0% power
		feederMotor.set(ControlMode.PercentOutput, 0.0);
	}
}
