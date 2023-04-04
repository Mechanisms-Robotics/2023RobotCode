package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDWrapper extends SubsystemBase {
	private static final double BLINK_TIME = 0.25; // seconds

	private final DigitalOutput m_ledRed = new DigitalOutput(1);
	private final DigitalOutput m_ledGreen = new DigitalOutput(3);
	private final DigitalOutput m_ledBlue = new DigitalOutput(2);

	private boolean[] m_color = new boolean[] {false, false, false};

	private final Timer m_blinkTimer = new Timer();
	private boolean m_blinking = false;

	public LEDWrapper() {}

	public void setColor(boolean[] color) {
		m_color = new boolean[] {!color[0], !color[1], !color[2]};
	}

	public void setBlinking(boolean blinking) {
		m_blinking = blinking;

		if (m_blinking) {
			m_blinkTimer.start();
		} else {
			m_blinkTimer.stop();
			m_blinkTimer.reset();
		}
	}

	public void turnOff() {
		setColor(new boolean[] {true, true, true});
		setBlinking(false);
	}

	@Override
	public void periodic() {
		if (m_blinking) {
			if (!m_blinkTimer.hasElapsed(BLINK_TIME)) {
				m_ledRed.set(m_color[0]);
				m_ledGreen.set(m_color[1]);
				m_ledBlue.set(m_color[2]);
			} else if (!m_blinkTimer.hasElapsed(BLINK_TIME * 2.0)) {
				m_ledRed.set(true);
				m_ledGreen.set(true);
				m_ledBlue.set(true);
			} else {
				m_blinkTimer.restart();
			}
		} else {
			m_ledRed.set(m_color[0]);
			m_ledGreen.set(m_color[1]);
			m_ledBlue.set(m_color[2]);
		}
	}
}
