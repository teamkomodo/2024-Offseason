package frc.robot.util;

import com.revrobotics.SparkPIDController;

public class Util {

    public static double translationCurve(double input) {
        return Math.pow(Math.abs(input), 1.5) * Math.signum(input);
        // double magnitude = Math.abs(input);
        // if(magnitude < 0.5) {
        //     return Math.signum(input) * (Math.pow(magnitude, 3) + .25 * input);
        // }
        
        // return Math.signum(input) * (-1 * Math.pow(magnitude, 3) + 3 * Math.pow(magnitude, 2) - 1.25 * magnitude + .25);
    }

    public static double steerCurve(double input) {
        return Math.pow(input, 3);
    }

    public static void setPidController(SparkPIDController pidController, PIDGains pid) {
        pidController.setP(pid.p);
        pidController.setI(pid.i);
        pidController.setD(pid.d);
        pidController.setIMaxAccum(pid.maxIAccum, 0);
    }

    public static void setPidController(SparkPIDController pidController, PIDGains pid, double iZone, double FF, double minOutput, double maxOutput) {
        setPidController(pidController, pid);
        pidController.setIZone(iZone);
        pidController.setFF(FF);
        pidController.setOutputRange(minOutput, maxOutput);
    }

}
