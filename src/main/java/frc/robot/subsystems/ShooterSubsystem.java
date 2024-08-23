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
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
    private final DoublePublisher shooterMotorSpeedPublisher = shooterTable.getDoubleTopic("shooterMotorSpeed").publish();
    private final DoublePublisher shooterMotorPositionPublisher = shooterTable.getDoubleTopic("shooterMotorPosition").publish();

    private final NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("indexer");
    private final DoublePublisher indexerMotorSpeedPublisher = indexerTable.getDoubleTopic("indexerMotorSpeed").publish();
    private final DoublePublisher indexerMotorPositionPublisher = indexerTable.getDoubleTopic("indexerMotorPosition").publish();

    private final CANSparkMax shooterMotor;
    private final SparkPIDController shooterPidController;
    private final RelativeEncoder shooterEncoder;
    private final CANSparkMax shooterMotor2;

    private PIDGains shooterPID = new PIDGains(1, 0, 0);
    private double shooterMaxIAccum = 0;
    private double shooterIZone = 1;
    private double shooterFF = 0.00022;
    private double shooterMinOutput = -1;
    private double shooterMaxOutput = 1;
    private double shooterMotorSpeed = 0;
    private double shooterMotorPosition = 0;

    private final CANSparkMax indexerMotor;
    private final SparkPIDController indexerPidController;
    private final RelativeEncoder indexerEncoder;
    private final CANSparkMax indexerMotor2;

    private PIDGains indexerPID = new PIDGains(0.1, 0, 0);
    private double indexerMaxIAccum = 0;
    private double indexerMotorSpeed = 0;
    private double indexerMotorPosition = 0;
  
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
        setPidController(shooterPidController, shooterPID, shooterMaxIAccum, shooterIZone, shooterFF, shooterMinOutput, shooterMaxOutput);
        setShooterMotor(0, ControlType.kDutyCycle);

        indexerMotor = new CANSparkMax(INDEXER_MOTOR_1_ID, MotorType.kBrushless);
        indexerMotor.setInverted(false);
        indexerMotor.setSmartCurrentLimit(30);

        indexerEncoder = indexerMotor.getEncoder();
        indexerEncoder.setPosition(0);

        indexerMotor2 = new CANSparkMax(INDEXER_MOTOR_2_ID, MotorType.kBrushless);
        indexerMotor2.setSmartCurrentLimit(30);
        indexerMotor2.setInverted(true);
        indexerMotor2.follow(indexerMotor, true);

        indexerPidController = indexerMotor.getPIDController();
        setPidController(indexerPidController, indexerPID, indexerMaxIAccum);
        setIndexerMotor(0, ControlType.kDutyCycle);
      }
  
    @Override
    public void periodic() {
        smoothCurrent = smoothCurrent * filterConstant + shooterMotor.getOutputCurrent() * (1-filterConstant);
        updateTable();
    }
  
    public void teleopInit() {
        setShooterMotor(0, ControlType.kDutyCycle);
        setIndexerMotor(0, ControlType.kDutyCycle);
    }
  
    public double getSmoothCurrent() {
        return smoothCurrent;
    }

    private void updateTable() {
        shooterMotorSpeedPublisher.set(shooterMotorSpeed);
        shooterMotorPositionPublisher.set(shooterMotorPosition);
        indexerMotorSpeedPublisher.set(indexerMotorSpeed);
        indexerMotorPositionPublisher.set(indexerMotorPosition);
    }
 
    public void setShooterMotorPosition(double position) {
        setShooterMotor(position, ControlType.kPosition);
    }
  
    public void setShooterMotorDutyCycle(double dutyCycle) {
        System.out.println("ran motor");
        setShooterMotor(dutyCycle, ControlType.kDutyCycle);
    }
  
    public void setShooterMotorVelocity(double velocity) {
        setShooterMotor(velocity, ControlType.kVelocity);
    }
  
    public void holdShooterMotorPosition() {
        setShooterMotor(shooterEncoder.getPosition(), ControlType.kPosition);
    }
  
    public double getShooterCurrent() {
        return shooterMotor.getOutputCurrent();
    }

    private void setShooterMotor(double value, ControlType type) {
        if (type == ControlType.kDutyCycle || type == ControlType.kVelocity) {
            shooterMotorSpeed = value;
        } else if (type == ControlType.kPosition) {
            shooterMotorPosition = value;
        }
        shooterPidController.setReference(value, type);
    }

    public void setIndexerMotorPosition(double position) {
        setIndexerMotor(position, ControlType.kPosition);
    }
  
    public void setIndexerMotorDutyCycle(double dutyCycle) {
        setIndexerMotor(dutyCycle, ControlType.kDutyCycle);
    }
  
    public void setIndexerMotorVelocity(double velocity) {
        setIndexerMotor(velocity, ControlType.kVelocity);
    }
  
    public void holdIndexerMotorPosition() {
        setIndexerMotor(shooterEncoder.getPosition(), ControlType.kPosition);
    }
  
    public double getIndexerCurrent() {
        return indexerMotor.getOutputCurrent();
    }

    private void setIndexerMotor(double value, ControlType type) {
        if (type == ControlType.kDutyCycle || type == ControlType.kVelocity) {
            indexerMotorSpeed = value;
        } else if (type == ControlType.kPosition) {
            indexerMotorPosition = value;
        }
        indexerPidController.setReference(value, type);
    }

    private void setPidController(SparkPIDController pidController, PIDGains pid, double iMaxAccum) {
        pidController.setP(pid.kP);
        pidController.setI(pid.kI);
        pidController.setD(pid.kD);
        pidController.setIMaxAccum(iMaxAccum, 0);
    }

    private void setPidController(SparkPIDController pidController, PIDGains pid, double iMaxAccum, double iZone, double FF, double minOutput, double maxOutput) {
        setPidController(pidController, pid, iMaxAccum);
        pidController.setP(iZone);
        pidController.setI(FF);
        pidController.setOutputRange(minOutput, maxOutput);
    }
}
