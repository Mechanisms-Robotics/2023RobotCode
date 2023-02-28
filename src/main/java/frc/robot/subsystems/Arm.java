package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    // TODO: find positions and extension positions
    private enum Position {
        High(3000, 0),
        Middle(3000, 0),
        Stowed(3000, 0);

        private final double armPosition;
        private final double extendPosition;

        Position(double armPosition, double extendPosition) {
            this.armPosition = armPosition;
            this.extendPosition = extendPosition;
        }
    }

    private static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration();

    private static final double MAX_GRAVITY_FF = 0.085;

    private static final double TICKS_PER_DEGREE = (2048.0 / 360.0) * 89.89;

    private static final double ARM_HORIZONTAL_POSITION = TICKS_PER_DEGREE * 33 + 36482;

    private static final double kP = 0.0;
    private static final double kD = 0.0;

    static {
        ARM_MOTOR_CONFIG.reverseSoftLimitEnable = true;
        ARM_MOTOR_CONFIG.forwardSoftLimitEnable = true;
        ARM_MOTOR_CONFIG.reverseSoftLimitThreshold = TICKS_PER_DEGREE * 33 + 2095;
        ARM_MOTOR_CONFIG.forwardSoftLimitThreshold = TICKS_PER_DEGREE * 33 + 46643;

        ARM_MOTOR_CONFIG.motionCruiseVelocity = 1000;
        ARM_MOTOR_CONFIG.motionAcceleration = 1000;

        ARM_MOTOR_CONFIG.neutralDeadband = 0.001;
    }

    private final WPI_TalonFX armMotor = new WPI_TalonFX(50);
    private final WPI_TalonFX extenderMotor = new WPI_TalonFX(51);

    public Arm() {
        armMotor.configAllSettings(ARM_MOTOR_CONFIG);

        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.config_kP(0, kP);
        armMotor.config_kD(0, kD);
    }

    public void setOpenLoop(double percentOutput) {
        armMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void periodic() {
        double radians =
                Math.toRadians(
                        (armMotor.getSelectedSensorPosition()
                                - ARM_HORIZONTAL_POSITION)
                                / TICKS_PER_DEGREE);
        double cosRadians = Math.cos(radians);
        SmartDashboard.putNumber("Arm Rot", Math.toRadians(cosRadians));
        System.out.println("ARM ANGLE: " + armMotor.getSelectedSensorPosition() / TICKS_PER_DEGREE);
    }

    public void setClosedLoop(double position) {
        double radians =
                Math.toRadians(
                        (armMotor.getSelectedSensorPosition()
                                - ARM_HORIZONTAL_POSITION)
                                / TICKS_PER_DEGREE);
        double cosRadians = Math.cos(radians);
        double demandFF = MAX_GRAVITY_FF * Math.cos(armMotor.getSelectedSensorPosition() / TICKS_PER_DEGREE);

        armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, demandFF);
    }

    public void horizontal() {
        setClosedLoop(ARM_HORIZONTAL_POSITION);
    }

    public void zeroEncoder() {
        armMotor.setSelectedSensorPosition(TICKS_PER_DEGREE * 33);
    }

    public void stop() {
        setOpenLoop(0.0);
    }

}
