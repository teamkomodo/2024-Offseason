package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JointSubsystem;

public class ZeroJointCommand extends DynamicCommand {

    private final JointSubsystem jointSubsystem;

    public ZeroJointCommand(JointSubsystem jointSubsystem) {
        
        this.jointSubsystem = jointSubsystem;

        addRequirements(jointSubsystem);
    }

    @Override
    protected Command getCommand() {
        return jointSubsystem.zeroJointCommand();
    }
}