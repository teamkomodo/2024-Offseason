package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlinkinPattern;

import static frc.robot.Constants.*;

public class LEDSubsystem extends SubsystemBase {
    
    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2

    private Spark frameLights = new Spark(A_FRAME_LED_CHANNEL);

    private double framePattern = 0;

    public void setFramePattern(double pattern){
        frameLights.set(pattern);
    }

    public Command setTempFramePatternCommand(double pattern) {
        return Commands.runEnd(() -> setFramePattern(pattern), () -> setFramePattern(framePattern));
    }

    public Command setFramePatternCommand(double pattern) {
        return Commands.runOnce(() -> { setFramePattern(pattern); framePattern = pattern;});
    }

    public Command flashGreenCommand() {
        return Commands.sequence(
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setFramePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1)
        );
    }

    public Command flashRedCommand() {
        return Commands.sequence(
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_RED).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_RED).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_RED).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_RED).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setFramePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1)
        );
    }

    public Command flashBlueCommand() {
        return Commands.sequence(
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLUE).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLUE).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLUE).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLUE).withTimeout(0.1),
            setTempFramePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            setFramePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1)
        );
    }
}
