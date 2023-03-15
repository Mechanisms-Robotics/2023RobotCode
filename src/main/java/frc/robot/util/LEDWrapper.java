package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDWrapper extends SubsystemBase {
  private final DigitalOutput m_ledGreen = new DigitalOutput(3);
  private final DigitalOutput m_ledBlue = new DigitalOutput(2);

  public LEDWrapper() {}

  public void setColor(boolean green, boolean blue) {
    m_ledGreen.set(green);
    m_ledBlue.set(blue);
  }

  public void turnOff() {
    setColor(false, false);
  }
}
