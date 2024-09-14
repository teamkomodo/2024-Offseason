package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.robot.Constants.*;

public class FalconSwerveModule implements SwerveModule{

    private final double drivePositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> wheel travel in meters
    private final double driveVelocityConversionFactor = drivePositionConversionFactor * 10; // sensor ticks/100ms -> wheel speed in m/s

    private final double steerPositionConversionFactor = 2 * Math.PI * STEER_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> module rotation in radians
    // private final double steerVelocityConversionFactor = steerPositionConversionFactor * 10; // sensor ticks/100ms -> module rad/s

    private final double driveP = 1.0;
    private final double driveI = 0;
    private final double driveD = 0;

    private final double steerP = 2.0e-1;
    private final double steerI = 1.0e-6;
    private final double steerD = 1.0e-7;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerAbsoluteEncoder;

    private final DutyCycleOut driveOut = new DutyCycleOut(0);
    private final DutyCycleOut steerOut = new DutyCycleOut(0);

    private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    private final Slot0Configs driveConfig = talonFXConfigs.Slot0;
    private final Slot1Configs steerConfig = talonFXConfigs.Slot1;

    private final PIDController driveController = new PIDController(driveP, driveI, driveD);
    
    // TODO Measure This
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 3.0); // V/M/s

    private SwerveModuleState desiredState;
    private double adjustedDesiredAngle = 0;

    private double relativeSteerAdjustment = 0;
    private double relativeSteerAdjustmentFactor = 0.1;

    public FalconSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, ShuffleboardContainer container) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.steerMotor = new TalonFX(steerMotorId);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));

        if(steerAbsoluteEncoder.configGetMagnetOffset() != steerOffset)
            steerAbsoluteEncoder.configMagnetOffset(Math.toDegrees(steerOffset));

        configureMotors();
        //buildShuffleboard(container);
    }

    /*private void buildShuffleboard(ShuffleboardContainer container) {
        container.addNumber("Desired Velocity", () -> desiredState.speedMetersPerSecond);
        container.addNumber("Desired Rotation", () -> desiredState.angle.getDegrees());
        container.addNumber("Rotation Error", () -> Math.toDegrees(getSteerPosition() - adjustedDesiredAngle));
        container.addNumber("Velocity Error", () -> getDriveVelocity() - desiredState.speedMetersPerSecond);

        container.addNumber("Raw Absolute Rotation", () -> steerAbsoluteEncoder.getAbsolutePosition());
        // container.addNumber("Adjusted Absolute Rotation", () -> getAbsoluteModuleRotation().getDegrees());
        container.addNumber("Raw Relative Rotation", () -> Math.toDegrees(getSteerPosition()));
        // container.addNumber("Adjusted Relative Rotation", () -> getModuleRotation().getDegrees());
        container.addNumber("Velocity", () -> getDriveVelocity());
    }*/

    private void configureMotors() {

        //driveMotor.setSensorPhase(true); // invert the sensor
        driveMotor.setInverted(true); // invert the motor
        driveMotor.setNeutralMode(NeutralMode.Brake); // motor brakes when idle

        steerMotor.setSensorPhase(false);
        steerMotor.setInverted(false);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        steerConfig.kP = 0.11;
        steerConfig.kI = 0.5;
        steerConfig.kD = 0.001;
        steerMotor.getConfigurator().apply(steerConfig, 0.050);
        steerMotor.setSelectedSensorPosition(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()) / steerPositionConversionFactor);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation());

        final double driveOutput = driveController.calculate(getDriveVelocity(), optimizedState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        
        driveMotor.setControl(driveOut);

        double currentAngleMod = MathUtil.angleModulus(getModuleRotation().getRadians());
        double adjustedAngle = optimizedState.angle.getRadians() + getModuleRotation().getRadians() - currentAngleMod;

        if(optimizedState.angle.getRadians() - currentAngleMod > Math.PI) {
            adjustedAngle -= 2.0 * Math.PI;
        }else if(optimizedState.angle.getRadians() - currentAngleMod < -Math.PI) {
            adjustedAngle += 2.0 * Math.PI;
        }
        adjustedDesiredAngle = adjustedAngle;
        steerMotor.setControl(steerOut);
        //steerMotor.set(TalonFXControlMode.Position, 0);
        this.desiredState = optimizedState;

        //correctRelativeEncoder();
    }

    //@SuppressWarnings(value = { "unused" })
    private void correctRelativeEncoder() {

        double absoluteAngle = Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
        double currentAngleMod = MathUtil.angleModulus(getModuleRotation().getRadians());
        double adjustedAngle = absoluteAngle + getSteerPosition() - currentAngleMod;

        if(absoluteAngle - currentAngleMod > Math.PI) {
            adjustedAngle -= 2.0 * Math.PI;
        }else if(absoluteAngle - currentAngleMod < -Math.PI) {
            adjustedAngle += 2.0 * Math.PI;
        }

        double delta = adjustedAngle - getSteerPosition();
        if(delta > Math.PI)
            delta -= 2 * Math.PI;

        if(delta < -Math.PI)
            delta += 2 * Math.PI;

        relativeSteerAdjustment += delta * relativeSteerAdjustmentFactor;

    }

    public Rotation2d getModuleRotation() {
        return new Rotation2d(getSteerPosition() + relativeSteerAdjustment);
        // return new Rotation2d(MathUtil.angleModulus(steerRelativeEncoder.getPosition() + steerOffset + relativeSteerAdjustment)); // Handled by 
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())));
        // return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) + steerOffset));
    }

    private double getSteerPosition() {
        return steerMotor.getRotorPosition().getValueAsDouble() * steerPositionConversionFactor;
    }
    
    private double getDrivePosition() {
        return driveMotor.getRotorPosition().getValueAsDouble() * drivePositionConversionFactor;
    }

    private double getDriveVelocity() {
        return driveMotor.getRotorPosition().getValueAsDouble() * driveVelocityConversionFactor;
    }
}