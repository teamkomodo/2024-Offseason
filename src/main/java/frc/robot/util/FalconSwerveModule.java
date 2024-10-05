package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class FalconSwerveModule implements SwerveModule{

    private static final double drivePositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION; // motor rotations -> wheel travel in meters
    private static final double driveVelocityConversionFactor = drivePositionConversionFactor; // motor rotations / s -> wheel speed in m/s

    private static final double steerPositionConversionFactor = 2 * Math.PI * STEER_REDUCTION; // motor rotations -> module rotation in radians
    
    // Telemetry
    private final DoublePublisher velocityErrorPublisher;
    private final DoublePublisher rotationErrorPublisher;
    private final DoublePublisher dutyCyclePublisher;

    private final double driveP = 1;
    private final double driveI = 0;
    private final double driveD = 0;

    private final double steerP = 3;
    private final double steerI = 0;
    private final double steerD = 0;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerAbsoluteEncoder;

    private final PIDController driveController = new PIDController(driveP, driveI, driveD);
    
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 12d / MAX_ATTAINABLE_VELOCITY); // V/M/s

    private SwerveModuleState desiredState;

    private double relativeSteerAdjustment = 0;
    private double relativeSteerAdjustmentFactor = 0.1;

    public FalconSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, NetworkTable moduleNT) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.steerMotor = new TalonFX(steerMotorId);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));

        steerAbsoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withMagnetOffset(steerOffset / (2 * Math.PI))
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)); // CANCoder outputs between (-0.5, 0.5)

        configureMotors();

        // Telemetry
        velocityErrorPublisher = moduleNT.getDoubleTopic("velocityerror").publish();
        rotationErrorPublisher = moduleNT.getDoubleTopic("rotationerror").publish();
        dutyCyclePublisher = moduleNT.getDoubleTopic("dutycycle").publish();
    }

    public void updateTelemetry() {
        velocityErrorPublisher.set(desiredState.speedMetersPerSecond - getDriveVelocity());
        rotationErrorPublisher.set(desiredState.angle.getRadians() - getModuleRotation().getRadians());
        dutyCyclePublisher.set(driveMotor.getDutyCycle().getValueAsDouble());
    }

    private void configureMotors() {

        driveMotor.setInverted(true); // invert the motor
        driveMotor.setNeutralMode(NeutralModeValue.Brake); // motor brakes when idle

        steerMotor.setInverted(true);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

        steerMotor.setPosition(-getAbsoluteModuleRotation().getRadians() / steerPositionConversionFactor); // negative becuase relative encoder is reversed
        steerMotor.getConfigurator().apply(new Slot0Configs().withKP(steerP).withKI(steerI).withKD(steerD));

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation()); // Doesn't matter if supplied rotation is upwrapped (I think)

        final double driveOutput = driveController.calculate(getDriveVelocity(), optimizedState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        
        driveMotor.setVoltage((driveOutput + driveFeedforward));

        // angle from kinematics is wrapped, we need to convert it to closest equivalent angle to current module rotation
        double minInputAngle = getModuleRotation().getRadians() - Math.PI;
        double maxInputAngle = getModuleRotation().getRadians() + Math.PI;
        double inputAngle = MathUtil.inputModulus(optimizedState.angle.getRadians(), minInputAngle, maxInputAngle);

        steerMotor.setControl(new PositionVoltage(-inputAngle / steerPositionConversionFactor)); // Negtaive because motor is inverted
        this.desiredState = optimizedState;
    }

    @SuppressWarnings(value = { "unused" })
    private void correctRelativeEncoder() {
        double delta = getAbsoluteModuleRotation().getRadians()-getModuleRotation().getRadians();
        if(delta > Math.PI)
            delta -= 2 * Math.PI;

        if(delta < -180)
            delta += 2 * Math.PI;

        relativeSteerAdjustment += delta * relativeSteerAdjustmentFactor;
    }

    public Rotation2d getModuleRotation() {
        return new Rotation2d(getSteerPosition() + relativeSteerAdjustment);
        // return new Rotation2d(MathUtil.angleModulus(steerRelativeEncoder.getPosition() + steerOffset + relativeSteerAdjustment)); // Handled by 
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
        // return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) + steerOffset));
    }

    private double getSteerPosition() {
        return -steerMotor.getPosition().getValueAsDouble() * steerPositionConversionFactor; // negative because motor is inverted
    }
    
    private double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * drivePositionConversionFactor;
    }

    private double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * driveVelocityConversionFactor;
    }

    @Override
    public void runForward(double voltage) {
        steerMotor.setControl(new PositionVoltage(0));
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void runRotation(double voltage) {
        driveMotor.setControl(new PositionVoltage(0));
        steerMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // never used 
    }
}