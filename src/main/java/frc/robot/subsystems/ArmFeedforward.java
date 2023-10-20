package frc.robot.subsystems;

import java.util.function.Supplier;

public class ArmFeedforward {
    private final Supplier<Double> comDistance;
    private final Supplier<Double> theta;
    private final double kG;

    public ArmFeedforward(Supplier<Double> comDistance, Supplier<Double> theta, double kG) {
        this.comDistance = comDistance;
        this.theta = theta;
        this.kG = kG;
    }
    public double calculate() {
        return comDistance.get() * Math.sin(theta.get()) * kG;
    }
}
