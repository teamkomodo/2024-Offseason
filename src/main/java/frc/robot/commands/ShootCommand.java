package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.JointSubsystem;

public class ShootCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final JointSubsystem jointSubsystem;

    public ShootCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, JointSubsystem jointSubsystem) {
        
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.jointSubsystem = jointSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem, jointSubsystem);
    }

    @Override
    protected Command getCommand() {
        // if(!jointSubsystem.getZeroed()) {
        //     return Commands.sequence(
        //         jointSubsystem.zeroJointCommand()
        //     );
        // }

        return new SequentialCommandGroup(
            // jointSubsystem.shootingPositionCommand(),
            // Commands.waitSeconds(0.3),
            intakeSubsystem.shoot(),
            Commands.waitSeconds(0.3),
            shooterSubsystem.stopShooter()//,
            // Commands.waitSeconds(0.3),
            // jointSubsystem.stowPositionCommand()
        );
    }
}