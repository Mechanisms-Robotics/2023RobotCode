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

		PhotonTrackedTarget bestTarget = null;

		if (result.hasTargets()) {
			for (int i = 0; i < result.getTargets().size(); i++) {
				if (bestTarget != null) {
					if (Math.abs(result.getTargets().get(i).getYaw())
									< Math.abs(bestTarget.getYaw())
							&& result.getTargets().get(i).getPitch() < 0.0) {
						bestTarget = result.getTargets().get(i);
					}
				} else {
					if (result.getTargets().get(i).getPitch() < 0.0) {
						bestTarget = result.getTargets().get(i);
					}
				}
			}
		}

		return bestTarget;
	}

	public void setLEDs(boolean on) {
		m_limelight.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
	}
}
