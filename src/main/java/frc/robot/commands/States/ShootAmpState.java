package frc.robot.commands.States;

import static frc.robot.Constants.INDEXER_SPEED;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TurboTakeSubsystem;

public class ShootAmpState extends Command{
    public TurboTakeSubsystem turbotakeSubsystem = new TurboTakeSubsystem();

    public ShootAmpState(TurboTakeSubsystem turboTakeSubsystem){
        addRequirements(turbotakeSubsystem);
    }

    protected Command getCommand(){

        return new SequentialCommandGroup(
            turbotakeSubsystem.runOnce(() -> turbotakeSubsystem.setIndexerVelocity(-INDEXER_SPEED)),
            new WaitCommand(0.6),
            turbotakeSubsystem.runOnce(() -> turbotakeSubsystem.setIndexerVelocity(0))
        );
    }
}