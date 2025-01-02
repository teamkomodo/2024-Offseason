package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class IntakeCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final JointSubsystem jointSubsystem;
    private LEDSubsystem ledSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, JointSubsystem jointSubsystem, LEDSubsystem ledSubsystem) {
        
        this.intakeSubsystem = intakeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem, jointSubsystem, ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!jointSubsystem.getZeroed()) {
            return Commands.sequence(
                jointSubsystem.zeroJointCommand()
            );
        }

        if(!intakeSubsystem.getNoteLoaded() || !intakeSubsystem.getNoteIntaked()) {
            return new SequentialCommandGroup(
                jointSubsystem.stowPositionCommand(),
                Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0.5)),
                Commands.waitUntil(() -> intakeSubsystem.getNoteIntaked()),
                Commands.runOnce(() -> ledSubsystem.flashGreenCommand()),
                Commands.waitUntil(() -> intakeSubsystem.getNoteLoaded()),
                Commands.runOnce(() -> intakeSubsystem.holdIntake())
            );
        }

        return null;
    }
}