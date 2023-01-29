package com.swervedrivespecialties.swervelib;

public final class SimpleFeedforwardConstants {
    public double ks;
    public double kv;
    public double ka;

    public SimpleFeedforwardConstants() {}

    public SimpleFeedforwardConstants(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }
}
