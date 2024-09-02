package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JointSubsystem;


public class ZeroJointCommand1 extends Command {

    private final JointSubsystem jointSubsystem;
    private final double velocity;

    public ZeroJointCommand1(JointSubsystem jointSubsystem, double velocity) {
        this.jointSubsystem = jointSubsystem;
        this.velocity = velocity;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (!jointSubsystem.getZeroed()) {
            jointSubsystem.setMotorVelocity(velocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        jointSubsystem.holdMotorPosition();
    }
    
}