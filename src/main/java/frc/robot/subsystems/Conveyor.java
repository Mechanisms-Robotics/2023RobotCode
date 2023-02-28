package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

    private static final double CONVEYOR_CONVEY_SPEED = 0.45;
    private static final double CONVEYOR_UNJAM_SPEED = -0.3;
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

        conveyorMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    }

    private void setOpenLoop(double percentOutput) {
        conveyorMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void convey() {
        setOpenLoop(CONVEYOR_CONVEY_SPEED);
    }

    public void unjam() {
        setOpenLoop(CONVEYOR_UNJAM_SPEED);
    }

    public void stop() {
        setOpenLoop(0.0);
    }
}
