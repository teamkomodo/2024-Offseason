// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class TestShooter extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final SparkMaxPIDController leftPidController;
    private final RelativeEncoder leftEncoder;

    private final CANSparkMax rightMotor;
    private final SparkMaxPIDController rightPidController;
    private final RelativeEncoder rightEncoder;
  
    private double p = 1.0;
    private double i = 0;
    private double d = 0;
    private double maxIAccum = 0;
  
    private double smoothCurrent = 0;
    private double filterConstant = 0.8;

    public TestShooter() {
        leftMotor = new CANSparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        leftMotor.setInverted(false);
        leftMotor.setSmartCurrentLimit(30);
        rightMotor = new CANSparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        rightMotor.setInverted(false);
        rightMotor.setSmartCurrentLimit(30);
        
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPosition(0);
        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPosition(0);

        leftPidController = leftMotor.getPIDController();
        leftPidController.setP(p);
        leftPidController.setI(i);
        leftPidController.setD(d);
        leftPidController.setIMaxAccum(maxIAccum, 0);
        rightPidController = rightMotor.getPIDController();
        rightPidController.setP(p);
        rightPidController.setI(i);
        rightPidController.setD(d);
        rightPidController.setIMaxAccum(maxIAccum, 0);
        setMotors(0, ControlType.kDutyCycle);
    }
    
    @Override
    public void periodic() {
        smoothCurrent = smoothCurrent * filterConstant + motor.getOutputCurrent() * (1-filterConstant);
    }
    
    public void teleopInit() {
        setMotors(0, ControlType.kDutyCycle);
    }

    public void setMotorDutyCycle(double dutyCycle) {
        setMotors(dutyCycle, ControlType.kDutyCycle);
    }
    
    public void setMotorVelocity(double velocity) {
        setMotors(velocity, ControlType.kVelocity);
    }
    
    public double getLeftMotorCurrent() {
        return leftMotor.getOutputCurrent();
    }

    public double getRightMotorCurrent() {
        return rightMotor.getOutputCurrent();
    }

    public double getCurrent() {
        return (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2;
    }
    
    public double getSmoothCurrent() {
        return smoothCurrent;
    }

    private void setMotors(double value, ControlType type) {
        leftPidController.setReference(value, type);
        rightPidController.setReference(value, type);
    }
}
