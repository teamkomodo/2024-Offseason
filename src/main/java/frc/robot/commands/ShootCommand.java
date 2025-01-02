package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ShootCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final JointSubsystem jointSubsystem;
    private final LEDSubsystem ledSubsystem;

    public ShootCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, JointSubsystem jointSubsystem, LEDSubsystem ledSubsystem) {
        
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem, jointSubsystem, ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!jointSubsystem.getZeroed()) {
            return Commands.sequence(
                jointSubsystem.zeroJointCommand()
            );
        }

        if(intakeSubsystem.getNoteLoaded()) {
            return new SequentialCommandGroup(
                jointSubsystem.shootingPositionCommand(),
                new WaitCommand(0.2),
                Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(-0.5)),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0.5)),
                Commands.waitSeconds(1.2),
                Commands.runOnce(() -> ledSubsystem.flashBlueCommand()),
                Commands.runOnce(() -> intakeSubsystem.holdIntake()),
                Commands.runOnce(() -> intakeSubsystem.noteLoaded = false),
                Commands.runOnce(() -> intakeSubsystem.noteIntaked = false),
                shooterSubsystem.stopShooter(),
                new WaitCommand(0.1),
                jointSubsystem.stowPositionCommand(),
                jointSubsystem.zeroJointCommand()
            );
        }

        return null;
    }
}