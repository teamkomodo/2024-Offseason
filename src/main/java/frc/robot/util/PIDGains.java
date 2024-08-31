package frc.robot.util;

public class PIDGains {
    
    public final double p;
    public final double i;
    public final double d;
    public final double maxIAccum;

    public PIDGains(double p, double i, double d) {
        this(p, i, d, 0);
    }

    public PIDGains(double p, double i, double d, double maxIAccum) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxIAccum = maxIAccum;
    }
    
    public PIDGains(double p, double i, double d, double maxIAccum, double a, double b, double c, double e) {
        this(0, 0, 0, 0);
    }
    
}
