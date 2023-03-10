package frc.robot.commands.goalTracker;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.GoalTracker;
import frc.robot.util.GoalTracker.TrackingMode;

public class SetTrackingMode extends InstantCommand {
	public SetTrackingMode(GoalTracker goalTracker, TrackingMode trackingMode) {
		super(() -> goalTracker.setTrackingMode(trackingMode));
	}
}
