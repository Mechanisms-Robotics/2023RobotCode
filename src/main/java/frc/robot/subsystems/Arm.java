package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

	private static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration ARM_EXTENDER_MOTOR_CONFIG =
			new TalonFXConfiguration();

	private static final double START_EXTENSION_POSITION = -6341;

	private static final double START_ARM_POSITION = 19000;

	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;

	private static final double ALLOWABLE_PIVOT_ERROR = 1000;
	private static final double ALLOWABLE_EXTENSION_ERROR = 1000;

	private static final double kP = 0.2; // 0.2
	private static final double kD = 0.0;
	private static final double kF = 0.0; // 0.01

	private static final double extenderKP = 1.0; // 0.8

	static {
		ARM_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_MOTOR_CONFIG.reverseSoftLimitThreshold = TICKS_PER_DEGREE * 33.0;
		ARM_MOTOR_CONFIG.forwardSoftLimitThreshold = 75000;

		ARM_MOTOR_CONFIG.motionCruiseVelocity = 20000;
		ARM_MOTOR_CONFIG.motionAcceleration = 15000; // 30000
		ARM_MOTOR_CONFIG.motionCurveStrength = 8;

		ARM_MOTOR_CONFIG.neutralDeadband = 0.001;

		ARM_EXTENDER_MOTOR_CONFIG.reverseSoftLimitEnable = true;
		ARM_EXTENDER_MOTOR_CONFIG.forwardSoftLimitEnable = true;
		ARM_EXTENDER_MOTOR_CONFIG.reverseSoftLimitThreshold = -17500;
		ARM_EXTENDER_MOTOR_CONFIG.forwardSoftLimitThreshold = 0;

		ARM_EXTENDER_MOTOR_CONFIG.motionCruiseVelocity = 15000;
		ARM_EXTENDER_MOTOR_CONFIG.motionAcceleration = 10000;
		ARM_EXTENDER_MOTOR_CONFIG.motionCurveStrength = 2;

		ARM_EXTENDER_MOTOR_CONFIG.voltageCompSaturation = 10.0;
	}

	private final WPI_TalonFX armMotor = new WPI_TalonFX(51);
	private final WPI_TalonFX extenderMotor = new WPI_TalonFX(52);

	private final double[] desiredPosition = {0.0, 0.0};

	private boolean zeroed = false;

	public Arm() {
		armMotor.configFactoryDefault();
		armMotor.configAllSettings(ARM_MOTOR_CONFIG);

		armMotor.setNeutralMode(NeutralMode.Brake);

		armMotor.config_kP(0, kP);
		armMotor.config_kD(0, kD);
		armMotor.config_kF(0, kF);

		armMotor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 15.0, 13.0, 1.0));

		armMotor.configNeutralDeadband(0.001);
		armMotor.configAllowableClosedloopError(0, 0.0);
		armMotor.setInverted(true);

		extenderMotor.configAllSettings(ARM_EXTENDER_MOTOR_CONFIG);

		extenderMotor.setNeutralMode(NeutralMode.Brake);

		extenderMotor.config_kP(0, extenderKP);
		extenderMotor.config_kI(0, 0.0);
		extenderMotor.config_kD(0, 0.0);

		extenderMotor.configNeutralDeadband(0.02);

		extenderMotor.configAllowableClosedloopError(0, 0.0); // 75

		extenderMotor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 20.0, 18.0, 1.0));

		extenderMotor.configSetParameter(ParamEnum.eContinuousCurrentLimitAmps, 30, 30, 0);

		extenderMotor.enableVoltageCompensation(true);

		armMotor.selectProfileSlot(0, 0);
		extenderMotor.selectProfileSlot(0, 0);
	}

	public void setOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		armMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	public void setExtensionOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		extenderMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Arm Zeroed", zeroed);
	}

	public void init() {
		if (zeroed) {
			System.out.println("ALREADY ZEROED");
			return;
		}

		armMotor.setSelectedSensorPosition(START_ARM_POSITION);
		extenderMotor.setSelectedSensorPosition(START_EXTENSION_POSITION);

		zeroed = true;
	}

	public void setClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		desiredPosition[0] = position;

		armMotor.set(ControlMode.MotionMagic, position);
	}

	public void setExtensionClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		desiredPosition[1] = position;

		extenderMotor.set(ControlMode.MotionMagic, position);
	}

	public boolean isAtPosition() {
		return Math.abs(armMotor.getSelectedSensorPosition() - desiredPosition[0])
				<= ALLOWABLE_PIVOT_ERROR;
	}

	public boolean isAtPosition(double pos) {
		return Math.abs(armMotor.getSelectedSensorPosition() - desiredPosition[0] / pos)
				<= ALLOWABLE_PIVOT_ERROR;
	}

	public boolean isAtMiddlePosition() {
		return Math.abs(armMotor.getSelectedSensorPosition() - desiredPosition[0] / 2)
				<= ALLOWABLE_PIVOT_ERROR;
	}

	public boolean extendAtPosition() {
		return Math.abs(extenderMotor.getSelectedSensorPosition() - desiredPosition[1])
				<= ALLOWABLE_EXTENSION_ERROR;
	}

	public boolean isExtended() {
		return extenderMotor.getSelectedSensorPosition() <= START_EXTENSION_POSITION;
	}

	public void zeroEncoder() {
		if (zeroed) {
			return;
		}

		armMotor.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		extenderMotor.setSelectedSensorPosition(0.0);
		zeroed = true;
	}

	public void stop() {
		setOpenLoop(0.0);
		setExtensionOpenLoop(0.0);
	}
}
