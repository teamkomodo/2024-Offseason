package frc.robot.util;


import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotController;

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

    private final PIDController driveController;

    
    // TODO Measure This
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 3.0); // V/M/s

    private SwerveModuleState desiredState;
    private double adjustedDesiredAngle = 0;

    private double relativeSteerAdjustment = 0;
    private double relativeSteerAdjustmentFactor = 0.1;

    // Telemetry
    private final DoublePublisher normalizedVelocityError;
    private final DoublePublisher rotationErrorPublisher;
    private final DoublePublisher dutyCyclePublisher;
    private final DoublePublisher velocityPublisher;

    private final DoubleEntry drivekPEntry;
    private final DoubleEntry drivekIEntry;
    private final DoubleEntry drivekDEntry;
    private final DoubleEntry drivekSEntry;
    private final DoubleEntry drivekVEntry;
    private final DoubleEntry drivekAEntry;

    private final DoubleEntry steerkPEntry;
    private final DoubleEntry steerkIEntry;
    private final DoubleEntry steerkDEntry;

    public FalconSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, PIDGains steerPIDGains, PIDGains drivePIDGains, FFGains driveFFGains, NetworkTable moduleNT) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.steerMotor = new TalonFX(steerMotorId);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));


        driveController = new PIDController(drivePIDGains.p, drivePIDGains.i, drivePIDGains.d);
        driveFeedforward = new SimpleMotorFeedforward(driveFFGains.s, driveFFGains.v, driveFFGains.a);


        steerAbsoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withMagnetOffset(steerOffset / (2 * Math.PI))
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)); // CANCoder outputs between (-0.5, 0.5)


        configureMotors();
        
        
        // Telemetry
        normalizedVelocityError = moduleNT.getDoubleTopic("normvelocityerror").publish();
        rotationErrorPublisher = moduleNT.getDoubleTopic("rotationerror").publish();
        dutyCyclePublisher = moduleNT.getDoubleTopic("dutycycle").publish();
        velocityPublisher = moduleNT.getDoubleTopic("velocity").publish();
        
        drivekPEntry = moduleNT.getDoubleTopic("tuning/drivekP").getEntry(drivePIDGains.p);
        drivekIEntry = moduleNT.getDoubleTopic("tuning/drivekI").getEntry(drivePIDGains.i);
        drivekDEntry = moduleNT.getDoubleTopic("tuning/drivekD").getEntry(drivePIDGains.d);
        drivekSEntry = moduleNT.getDoubleTopic("tuning/drivekS").getEntry(driveFFGains.s);
        drivekVEntry = moduleNT.getDoubleTopic("tuning/drivekV").getEntry(driveFFGains.v);
        drivekAEntry = moduleNT.getDoubleTopic("tuning/drivekA").getEntry(driveFFGains.a);

        steerkPEntry = moduleNT.getDoubleTopic("tuning/steerkP").getEntry(steerPIDGains.p);
        steerkIEntry = moduleNT.getDoubleTopic("tuning/steerkI").getEntry(steerPIDGains.i);
        steerkDEntry = moduleNT.getDoubleTopic("tuning/steerkD").getEntry(steerPIDGains.d);

        drivekPEntry.set(drivePIDGains.p);
        drivekIEntry.set(drivePIDGains.i);
        drivekDEntry.set(drivePIDGains.d);
        drivekSEntry.set(driveFFGains.s);
        drivekVEntry.set(driveFFGains.v);
        drivekAEntry.set(driveFFGains.a);

        steerkPEntry.set(steerPIDGains.p);
        steerkIEntry.set(steerPIDGains.i);
        steerkDEntry.set(steerPIDGains.d);
    }

    private void configureMotors() {

        //driveMotor.setSensorPhase(true); // invert the sensor
        driveMotor.setInverted(true); // invert the motor

        driveMotor.setInverted(false);

        driveConfig.kP = driveP;
        driveConfig.kI = driveI;
        driveConfig.kD = driveD;
        
        driveMotor.getConfigurator().apply(steerConfig, 0.050);
        driveMotor.setPosition(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()) / steerPositionConversionFactor);
        driveOut.OverrideBrakeDurNeutral = true;


        steerMotor.setInverted(false);

        steerConfig.kP = steerP;
        steerConfig.kI = steerI;
        steerConfig.kD = steerD;
        
        steerMotor.getConfigurator().apply(steerConfig, 0.050);
        steerMotor.setPosition(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()) / steerPositionConversionFactor);
        steerOut.OverrideBrakeDurNeutral = true;

    }

    public void updateTelemetry(){
        normalizedVelocityError.set((desiredState.speedMetersPerSecond - getDriveVelocity()) * Math.signum(desiredState.speedMetersPerSecond));
        rotationErrorPublisher.set(MathUtil.angleModulus(desiredState.angle.getRadians() - getModuleRotation().getRadians()));
        dutyCyclePublisher.set(driveMotor.get());
        velocityPublisher.set(getDriveVelocity(), RobotController.getFPGATime() - 200000);

        if(!TUNING_MODE)
            return;

        double newDrivekP = drivekPEntry.get();
        if(newDrivekP != driveController.getP()) driveController.setP(newDrivekP);

        double newDrivekI = drivekIEntry.get();
        if(newDrivekI != driveController.getI()) driveController.setI(newDrivekI);

        double newDrivekD = drivekDEntry.get();
        if(newDrivekD != driveController.getD()) driveController.setD(newDrivekD);

        double newDrivekS = drivekSEntry.get();
        double newDrivekV = drivekVEntry.get();
        double newDrivekA = drivekAEntry.get();

        if(newDrivekS != driveFeedforward.ks || newDrivekV != driveFeedforward.kv || newDrivekA != driveFeedforward.ka){
            driveFeedforward = new SimpleMotorFeedforward(newDrivekS, newDrivekV, newDrivekA);
        }

        double newSteerkP = steerkPEntry.get();
        if(newSteerkP != steerConfig.kP) {steerConfig.kP = newSteerkP;}

        double newSteerkI = steerkIEntry.get();
        if(newSteerkI != steerConfig.kI) {steerConfig.kI = newSteerkI;}

        double newSteerkD = steerkDEntry.get();
        if(newSteerkD != steerConfig.kD) {steerConfig.kD = newSteerkD;}

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation());


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

    public SwerveModuleState getDesiredState(){
        return desiredState;
    }

    @Override
    public void periodic() {
        final double driveOutput = driveController.calculate(driveMotor.getVelocity().getValueAsDouble(), desiredState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        //System.out.println(driveFeedforward);
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        steerMotor.setPosition(desiredState.angle.getRadians());
    }

    @Override
    public void runForward(double voltage) {
        driveMotor.setVoltage(voltage);
        steerMotor.setPosition(0);
    }

    @Override
    public void runRotation(double voltage) {
        driveMotor.set(0);
        steerMotor.setVoltage(voltage);
    }
}