package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight extends SubsystemBase {
	private static final PhotonCamera m_limelight = new PhotonCamera("limelight");

	public PhotonTrackedTarget getBestTarget() {
		PhotonPipelineResult result = m_limelight.getLatestResult();

		if (result.hasTargets()) {
			return result.getBestTarget();
		}

		return null;
	}

	public void setLEDs(boolean on) {
		m_limelight.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
	}
}
