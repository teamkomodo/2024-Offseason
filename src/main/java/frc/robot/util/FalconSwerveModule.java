package frc.robot.util;


import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
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
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class FalconSwerveModule implements SwerveModule{

    private final double drivePositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> wheel travel in meters
    private final double driveVelocityConversionFactor = drivePositionConversionFactor * 10; // sensor ticks/100ms -> wheel speed in m/s

    private final double steerPositionConversionFactor = 2 * Math.PI * STEER_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> module rotation in radians
    // private final double steerVelocityConversionFactor = steerPositionConversionFactor * 10; // sensor ticks/100ms -> module rad/s


    private final TalonFX driveMotor;

    private final TalonFX steerMotor;
    private final CANcoder steerAbsoluteEncoder;

    //*Added DutyCycles
    private final DutyCycleOut driveOut = new DutyCycleOut(0);
    private final DutyCycleOut steerOut = new DutyCycleOut(0);

    //*Added Configs
    private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    private final Slot0Configs driveConfig = talonFXConfigs.Slot0;
    private final Slot1Configs steerConfig = talonFXConfigs.Slot1;

    private final PIDController driveController; //moved definition to object initialization

    
    //TODO Measure This
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 3.0); // V/M/s

    private SwerveModuleState desiredState;
    private double adjustedDesiredAngle = 0;

    private double relativeSteerAdjustment = 0;
    private double relativeSteerAdjustmentFactor = 0.1;

    private final PositionVoltage anglePosition = new PositionVoltage(0);
    private Rotation2d lastAngle;

    //*Added Telemetry from NeoSwerveModule
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

    //*Changed what it takes in to be more in line with NeoSwerveModule
    public FalconSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, PIDGains steerPIDGains, PIDGains drivePIDGains, FFGains driveFFGains, NetworkTable moduleNT) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.steerMotor = new TalonFX(steerMotorId);
        
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));
        
        //*Removed configGetMagnetOffset because the method is depricated

        lastAngle = getState().angle;

        //*Defined driveController and FF
        driveController = new PIDController(drivePIDGains.p, drivePIDGains.i, drivePIDGains.d);
        driveFeedforward = new SimpleMotorFeedforward(driveFFGains.s, driveFFGains.v, driveFFGains.a);

        
        //*Configs the steerAbsoluteEncoder as it does in NeoSwerveModule
        steerAbsoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withMagnetOffset(steerOffset / (2 * Math.PI))
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)); // CANCoder outputs between (-0.5, 0.5)

        
        //*ConfigureMotors works as it does in NeoSwerveModule
        configureMotors(steerPIDGains);
        
        
        //*Added Telemetry
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

    private void configureMotors(PIDGains steerGains) {

        

        //driveMotor.setSensorPhase(true); // invert the sensor
        driveMotor.setInverted(true); // invert the motor
        //*Removed Neutrol Mode Brake for driveMotor

        //*This is probably from NeoSwerveModule but also is useless
        double wheelPositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION; // motor rotations -> wheel travel in meters
    
        //*Removed setSensorPhase as it was depricated
        steerMotor.setInverted(false);
        //*New way to set braking during Neutral
        steerOut.OverrideBrakeDurNeutral = true;

        //*New way to set PID for steerMotor
        steerConfig.kP = steerGains.p;
        steerConfig.kI = steerGains.i;
        steerConfig.kD = steerGains.d;
        
        steerMotor.getConfigurator().apply(steerConfig);
        
        

    }

    //*Added Telemetry
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

    public SwerveModuleState getDesiredState(){
        return desiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getModuleRotation());
    }

    //*Changed setDesiredState to the one used in NeoSwerveModule, which helps with Swerve Jitteriness and Gear Grinding
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation());
        this.desiredState = optimizedState;

    }

    //*Removed correctRelativeEncoder


    //*Added getValueAsDouble
    public Rotation2d getModuleRotation() {
        return new Rotation2d(steerAbsoluteEncoder.getPosition().getValueAsDouble() + relativeSteerAdjustment);
        // return new Rotation2d(MathUtil.angleModulus(steerRelativeEncoder.getPosition() + steerOffset + relativeSteerAdjustment)); // Handled by 
    }

    //*Added getValueAsDouble, also set it to same thing as getModuleRotation
    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(steerAbsoluteEncoder.getPosition().getValueAsDouble() + relativeSteerAdjustment);
        // return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) + steerOffset));
    }

    
    //*Added getValueAsDouble
    private double getDrivePosition() {
        return driveMotor.getRotorPosition().getValueAsDouble() * drivePositionConversionFactor;
    }

    //*Added getValueAsDouble
    private double getDriveVelocity() {
        return driveMotor.getRotorPosition().getValueAsDouble() * driveVelocityConversionFactor;
    }


    //*Added Periodic
    @Override
    public void periodic() {
        final double driveOutput = driveController.calculate(driveMotor.getVelocity().getValueAsDouble(), desiredState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        anglePosition.Position = (desiredState.angle.getDegrees() / 180) * 12.8;
        //System.out.println(driveFeedforward);
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        
        
        //steerMotor.setVoltage((desiredState.angle.getRadians()-steerMotor.getPosition().getValueAsDouble()%(2*Math.PI)));
        //setAngle(desiredState);
        steerMotor.setControl(anglePosition);

        //System.out.println(desiredState.angle.getRadians());
    }

    //*Added for SysID
    @Override
    public void runForward(double voltage) {
        driveMotor.setVoltage(voltage);
        steerMotor.setPosition(0);
    }

    //*Added for SysID
    @Override
    public void runRotation(double voltage) {
        driveMotor.set(0);
        steerMotor.setVoltage(voltage);
    }
}