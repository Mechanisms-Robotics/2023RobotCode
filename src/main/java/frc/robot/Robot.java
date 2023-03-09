// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AprilTagTracker;
import org.photonvision.common.hardware.VisionLEDMode;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		AprilTagTracker.getCamera().setLED(VisionLEDMode.kOff);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/** This method is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		m_robotContainer.m_intakeSubsystem.stopPivot();
		m_robotContainer.m_armSubsystem.stop();
		m_robotContainer.m_gripperSubsystem.stop();
	}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		//		m_robotContainer.m_intakeSubsystem.zeroEncoders();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		m_robotContainer.m_swerveSubsystem.setNeutralMode(NeutralMode.Brake);
		m_robotContainer.m_swerveSubsystem.zeroGyro();
	}

	/** This method is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		m_robotContainer.m_armSubsystem.stow();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		m_robotContainer.m_swerveSubsystem.setNeutralMode(NeutralMode.Coast);
	}

	/** This method is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();

		m_robotContainer.m_intakeSubsystem.zeroEncoders();
		m_robotContainer.m_armSubsystem.zeroEncoder();
		m_robotContainer.m_swerveSubsystem.zeroGyro();
		m_robotContainer.m_gripperSubsystem.zeroEncoder();
	}

	/** This method is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This method is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This method is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}
}
