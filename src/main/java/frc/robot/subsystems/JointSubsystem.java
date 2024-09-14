// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;
import frc.robot.util.Util;

import static frc.robot.Constants.*;

public class JointSubsystem extends SubsystemBase {
    private final NetworkTable jointTable = NetworkTableInstance.getDefault().getTable("joint");
    private final DoublePublisher jointMotorPositionPublisher = jointTable.getDoubleTopic("jointMotorPosition").publish();

    private final CANSparkMax jointMotor;
    private final CANSparkMax jointMotor2;
    private final SparkPIDController jointPidController;
    private final RelativeEncoder jointEncoder;
    private final DigitalInput jointLimitSwitch;

    private PIDGains jointPID = new PIDGains(0.1, 0, 0, 0);
    private boolean prevLimitSwitchValue;
    private boolean zeroed = false;

    private double jointMotorPosition = 0;
  
    public JointSubsystem() {
        jointMotor = new CANSparkMax(JOINT_MOTOR_1_ID, MotorType.kBrushless);
        jointMotor.setSmartCurrentLimit(50);
        jointMotor.setInverted(false);
        jointMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) JOINT_MIN_POSITION);
        jointMotor.setSoftLimit(SoftLimitDirection.kForward, (float) JOINT_MAX_POSITION);
        jointMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        jointMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        jointMotor2 = new CANSparkMax(JOINT_MOTOR_2_ID, MotorType.kBrushless);
        jointMotor2.setSmartCurrentLimit(50);
        jointMotor2.setInverted(true);
        jointMotor2.follow(jointMotor, true);

        jointEncoder = jointMotor.getEncoder();
        jointEncoder.setPosition(0);

        jointLimitSwitch = new DigitalInput(JOINT_ZERO_SWITCH_CHANNEL);
        prevLimitSwitchValue = jointLimitSwitch.get();
        
        jointPidController = jointMotor.getPIDController();
        Util.setPidController(jointPidController, jointPID);
        setMotor(0, ControlType.kDutyCycle);
      }
  
    @Override
    public void periodic() {
        updateTable();
        checkLimitSwitch();
    }
  
    public void teleopInit() {
        
    }

    private void updateTable() {
        jointMotorPositionPublisher.set(jointMotorPosition);
    }

    public void checkLimitSwitch() {
        if (getLimitSwitch() != prevLimitSwitchValue) {
            jointEncoder.setPosition(JOINT_MIN_POSITION);
            zeroed = true;
            if (!prevLimitSwitchValue) { // Only stop motor on leading edge, meaning does not sense magnet before.
                setMotorPosition(JOINT_MIN_POSITION);
            }
        }
        prevLimitSwitchValue = getLimitSwitch();
    }

    public boolean getLimitSwitch() {
        return !jointLimitSwitch.get(); // jointLimitSwitch.get() is weird
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
        setMotor(jointEncoder.getPosition(), ControlType.kPosition);
    }

    public boolean isZeroed() {
        return zeroed;
    }

    public Command stowPositionCommand() {
        return this.runOnce(() -> setMotorPosition(STOW_POSITION));
    }

    public Command shootingPositionCommand() {
        return this.runOnce(() -> setMotorPosition(SHOOTING_POSITION));
    }

    public Command zeroJointCommand() {
        return Commands.runEnd(() -> setMotorDutyCycle(-0.1), () -> setMotorDutyCycle(0), this).until(() -> (zeroed));
    }
    
    private void setMotor(double value, ControlType type) {
        if (type == ControlType.kPosition) {
            jointMotorPosition = value;
        }
        jointPidController.setReference(value, type);
    }
}
