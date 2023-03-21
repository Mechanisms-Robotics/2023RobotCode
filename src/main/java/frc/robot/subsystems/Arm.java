package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.State;

public class Arm extends SubsystemBase {

	public enum ArmState {
		Retracting,
		Pivoting,
		Deploying,
		Idle,
	}

	private static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration();
	private static final TalonFXConfiguration ARM_EXTENDER_MOTOR_CONFIG =
			new TalonFXConfiguration();

	private static final double STOWED_POSITION = -500;
	private static final double START_EXTENSION_POSITION = -6341;

	private static final double START_ARM_POSITION = 19000;

	private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;

	private static final double ALLOWABLE_PIVOT_ERROR = 1000;
	private static final double ALLOWABLE_EXTENSION_ERROR = 400;

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

		ARM_EXTENDER_MOTOR_CONFIG.motionCruiseVelocity = 10000;
		ARM_EXTENDER_MOTOR_CONFIG.motionAcceleration = 7000;
		ARM_EXTENDER_MOTOR_CONFIG.motionCurveStrength = 2;
	}

	private final WPI_TalonFX armMotorLeft = new WPI_TalonFX(50);
	private final WPI_TalonFX armMotorRight = new WPI_TalonFX(51);
	private final WPI_TalonFX extenderMotor = new WPI_TalonFX(52);

	private ArmState armState = ArmState.Idle;
	private final double[] desiredPosition = {0.0, 0.0};

	private boolean zeroed = false;

	private int selectedMotorNum = 1;
	private WPI_TalonFX selectedMotor = armMotorLeft;

	public Arm() {
		armMotorLeft.configFactoryDefault();
		armMotorLeft.configAllSettings(ARM_MOTOR_CONFIG);

		armMotorRight.configFactoryDefault();
		armMotorRight.configAllSettings(ARM_MOTOR_CONFIG);

		armMotorLeft.setNeutralMode(NeutralMode.Brake);
		armMotorRight.setNeutralMode(NeutralMode.Brake);

		armMotorLeft.config_kP(0, kP);
		armMotorLeft.config_kD(0, kD);
		armMotorLeft.config_kF(0, kF);

		armMotorRight.config_kP(0, kP);
		armMotorRight.config_kD(0, kD);
		armMotorRight.config_kF(0, kF);

		armMotorRight.setSensorPhase(false);

		armMotorLeft.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 15.0, 13.0, 1.0));

		armMotorRight.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, 15.0, 13.0, 1.0));

		armMotorLeft.configNeutralDeadband(0.001);
		armMotorLeft.configAllowableClosedloopError(0, 0.0);

		armMotorRight.configNeutralDeadband(0.001);
		armMotorRight.configAllowableClosedloopError(0, 0.0);

		armMotorRight.setInverted(TalonFXInvertType.OpposeMaster);

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

		armMotorLeft.selectProfileSlot(0, 0);
		armMotorRight.selectProfileSlot(0, 0);
		extenderMotor.selectProfileSlot(0, 0);
	}

	public void setOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		armMotorLeft.set(ControlMode.PercentOutput, percentOutput);
		armMotorRight.follow(armMotorLeft);
	}

	public void setExtensionOpenLoop(double percentOutput) {
		if (!zeroed) {
			return;
		}

		extenderMotor.set(ControlMode.PercentOutput, percentOutput);
	}

	@Override
	public void periodic() {
		switch (armState) {
			case Idle:
				idle();
				break;
			case Retracting:
				retract();
				break;
			case Pivoting:
				pivot();
				break;
			case Deploying:
				deploy();
				break;
		}

		SmartDashboard.putString("ArmState", armState.toString());
		SmartDashboard.putBoolean("ArmZeroed", zeroed);
	}

	public void init() {
		if (zeroed) {
			System.out.println("ALREADY ZEROED");
			return;
		}

		armMotorLeft.setSelectedSensorPosition(START_ARM_POSITION);
		armMotorRight.setSelectedSensorPosition(START_ARM_POSITION);
		extenderMotor.setSelectedSensorPosition(START_EXTENSION_POSITION);

		zeroed = true;
	}

	public void setArm(double armPosition, double extendPosition, int selectedMotor) {
		if (this.desiredPosition[0] != armPosition || this.desiredPosition[1] != extendPosition || selectedMotor != this.selectedMotorNum) {
			this.desiredPosition[0] = armPosition;
			this.desiredPosition[1] = extendPosition;

			this.selectedMotorNum = selectedMotor;

			if (this.selectedMotorNum == 1) {
				this.selectedMotor = armMotorLeft;
			} else {
				this.selectedMotor = armMotorRight;
			}

			retract();
		}
	}

	private void setClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		if (this.selectedMotorNum == 1) {
			armMotorLeft.setNeutralMode(NeutralMode.Brake);
			armMotorLeft.set(ControlMode.MotionMagic, position);

			armMotorRight.setNeutralMode(NeutralMode.Coast);
			armMotorRight.set(ControlMode.PercentOutput, 0.0);
		} else {
			armMotorLeft.setNeutralMode(NeutralMode.Coast);
			armMotorLeft.set(ControlMode.PercentOutput, 0.0);

			armMotorRight.setNeutralMode(NeutralMode.Brake);
			armMotorRight.set(ControlMode.MotionMagic, position);
		}
	}

	private void setExtensionClosedLoop(double position) {
		if (!zeroed) {
			return;
		}

		extenderMotor.set(ControlMode.MotionMagic, position);
	}

	private void retract() {
		if (DriverStation.isEnabled() && armState == ArmState.Retracting) {
			if (Math.abs(extenderMotor.getSelectedSensorPosition() - STOWED_POSITION)
					<= ALLOWABLE_EXTENSION_ERROR) {
				pivot();
			}

			return;
		}

		armState = ArmState.Retracting;
		setExtensionClosedLoop(STOWED_POSITION);
	}

	public void setState(ArmState state) {
		armState = state;
	}

	private void pivot() {
		if (armState == ArmState.Pivoting) {
			if (Math.abs(selectedMotor.getSelectedSensorPosition() - desiredPosition[0])
					<= ALLOWABLE_PIVOT_ERROR) {
				if (desiredPosition[1] != STOWED_POSITION) {
					deploy();
				} else {
					idle();
				}
			}

			return;
		}

		armState = ArmState.Pivoting;
		setClosedLoop(desiredPosition[0]);
	}

	private void deploy() {
		if (armState == ArmState.Deploying) {
			if (Math.abs(extenderMotor.getSelectedSensorPosition() - desiredPosition[1])
					<= ALLOWABLE_EXTENSION_ERROR) {
				idle();
			}

			return;
		}

		armState = ArmState.Deploying;

		setExtensionClosedLoop(desiredPosition[1]);
	}

	private void idle() {
		armState = ArmState.Idle;
	}

	public boolean isAtPosition() {
		return Math.abs(selectedMotor.getSelectedSensorPosition() - desiredPosition[0])
				<= ALLOWABLE_PIVOT_ERROR;
	}

	public boolean extendAtPosition() {
		return Math.abs(extenderMotor.getSelectedSensorPosition() - desiredPosition[1])
				<= ALLOWABLE_EXTENSION_ERROR;
	}

	public boolean isIdle() {
		return armState == ArmState.Idle;
	}

	public void zeroEncoder() {
		if (zeroed) {
			return;
		}

		armMotorLeft.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		armMotorRight.setSelectedSensorPosition(TICKS_PER_DEGREE * 33.0);
		extenderMotor.setSelectedSensorPosition(0.0);
		zeroed = true;
	}

	public void stop() {
		setOpenLoop(0.0);
		setExtensionOpenLoop(0.0);
	}

	public double[] getDesiredPosition() {
		return desiredPosition;
	}
}
