// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.BlinkinPattern;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

import java.lang.module.FindException;
import java.util.Map;

public class RobotContainer {  
    private final Field2d field2d = new Field2d();

    private final SendableChooser<Command> autoChooser;

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_XBOX_PORT);

    //private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final JointSubsystem jointSubsystem = new JointSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    public RobotContainer() {
        configureBindings();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }    
    
    private void configureBindings() {

        Trigger xButtonOperator = operatorController.x();
        Trigger yButtonOperator = operatorController.y();
        Trigger aButtonOperator = operatorController.a();
        Trigger bButtonOperator = operatorController.b();
        
        Trigger rightJoystickOperator = operatorController.rightStick();
        Trigger leftJoystickOperator = operatorController.leftStick();

        Trigger rightTriggerOperator = operatorController.rightTrigger();
        Trigger leftTriggerOperator = operatorController.leftTrigger();
        Trigger rightBumperOperator = operatorController.rightBumper();
        Trigger leftBumperOperator = operatorController.rightBumper();
        
        //functionality
        rightTriggerOperator.onTrue(shooterSubsystem.rampUpShooter());
        leftTriggerOperator.onTrue(shooterSubsystem.stopShooter());

        yButtonOperator.whileTrue(intakeSubsystem.intake());
        
        aButtonOperator.onTrue(new ShootCommand(intakeSubsystem, shooterSubsystem, jointSubsystem));
        bButtonOperator.onTrue(intakeSubsystem.eject());

        //drivetrain
		Trigger leftBumperDriver = driverController.leftBumper();
		leftBumperDriver.onTrue(Commands.runOnce(() -> {drivetrainSubsystem.zeroGyro();}));

		// deadbands are applied in command
		drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.joystickDriveCommand(
				() -> -driverController.getLeftY(), // -Y (up) on joystick is +X (forward) on robot
				() -> -driverController.getLeftX(), // -X (left) on joystick is +Y (left) on robot
				() -> driverController.getRightX() // -X (left) on joystick is +Theta (counter-clockwise) on robot
		));

    }

    public void teleopInit() {
        intakeSubsystem.holdIntake();
        jointSubsystem.zeroJointCommand();
    }
    
    public Command getAutonomousCommand() {
        return null;
        // if(autoChooser != null) {
        //    return autoChooser.getSelected();
        // }
    }

    private void registerNamedCommands() {
        
    }
}
