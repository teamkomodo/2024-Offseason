// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;
import frc.robot.util.Util;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private final DoublePublisher motorSpeedPublisher = shooterTable.getDoubleTopic("motorSpeed").publish();
    private final DoublePublisher motorPositionPublisher = shooterTable.getDoubleTopic("motorPosition").publish();

    private final CANSparkMax shooterMotor;
    private final SparkPIDController shooterPidController;
    private final RelativeEncoder shooterEncoder;
    private final CANSparkMax shooterMotor2;

    private PIDGains pid = new PIDGains(1, 0, 0, 0, 1, 0.00022, -1, 1);

    private double motorSpeed = 0;
    private double motorPosition = 0;
  
    private double smoothCurrent = 0;
    private double filterConstant = 0.8;
    
  
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        shooterMotor.setInverted(false);
        shooterMotor.setSmartCurrentLimit(50);

        shooterEncoder = shooterMotor.getEncoder();
        shooterEncoder.setPosition(0);

        shooterMotor2 = new CANSparkMax(SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        shooterMotor2.setSmartCurrentLimit(50);
        shooterMotor2.setInverted(true);
        shooterMotor2.follow(shooterMotor, true);
        
        shooterPidController = shooterMotor.getPIDController();
        Util.setPidController(shooterPidController, pid);
        setMotor(0, ControlType.kDutyCycle);
    }
  
    @Override
    public void periodic() {
        updateCurrent();
        updateTable();
    }
  
    public void teleopInit() {
        setMotor(0, ControlType.kDutyCycle);
    }

    private void updateTable() {
        motorSpeedPublisher.set(motorSpeed);
        motorPositionPublisher.set(motorPosition);
    }

    private void updateCurrent() {
        smoothCurrent = smoothCurrent * filterConstant + shooterMotor.getOutputCurrent() * (1-filterConstant);
    }
 
    public void setMotorPosition(double position) {
        setMotor(position, ControlType.kPosition);
    }
  
    public void setMotorDutyCycle(double dutyCycle) {
        setMotor(dutyCycle, ControlType.kDutyCycle);
    }
  
    public void setMotorVelocity(double velocity) {
        setMotor(velocity, ControlType.kVelocity);
    }
  
    public void holdMotorPosition() {
        setMotor(shooterEncoder.getPosition(), ControlType.kPosition);
    }

    private void setMotor(double value, ControlType type) {
        if (type == ControlType.kDutyCycle || type == ControlType.kVelocity)
            motorSpeed = value;
        else if (type == ControlType.kPosition)
            motorPosition = value;
        shooterPidController.setReference(value, type);
    }

    public double getCurrent() {
        return shooterMotor.getOutputCurrent();
    }

    public double getSmoothCurrent() {
        return smoothCurrent;
    }
}
