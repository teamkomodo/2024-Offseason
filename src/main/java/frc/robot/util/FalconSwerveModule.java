package frc.robot.util;



import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.robot.Constants.*;

public class FalconSwerveModule implements SwerveModule{

    private final double drivePositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> wheel travel in meters
    private final double driveVelocityConversionFactor = drivePositionConversionFactor * 10; // sensor ticks/100ms -> wheel speed in m/s

    private final double steerPositionConversionFactor = 2 * Math.PI * STEER_REDUCTION / TALON_FX_TICKS_PER_ROTATION; // sensor ticks -> module rotation in radians
    // private final double steerVelocityConversionFactor = steerPositionConversionFactor * 10; // sensor ticks/100ms -> module rad/s

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

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerAbsoluteEncoder;


    private final PIDController driveController;

    private final PIDController steerController;
        
    // TODO Measure This
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 3.0); // V/M/s

    private double relativeSteerAdjustment = 0;
    private double relativeSteerAdjustmentFactor = 0.1;

    private SwerveModuleState desiredState;
    private double adjustedDesiredAngle = 0;

    public FalconSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, PIDGains steerPIDGains, PIDGains drivePIDGains, FFGains driveFFGains, NetworkTable moduleNT) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.steerMotor = new TalonFX(steerMotorId);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));
        
        driveController = new PIDController(drivePIDGains.p, drivePIDGains.i, drivePIDGains.d);
        driveFeedforward = new SimpleMotorFeedforward(driveFFGains.s, driveFFGains.v, driveFFGains.a);

        if(steerAbsoluteEncoder.configGetMagnetOffset() != steerOffset){
            steerAbsoluteEncoder.configMagnetOffset(Math.toDegrees(steerOffset));
        }

        steerAbsoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs()
            .withMagnetOffset(steerOffset / (2 * Math.PI))
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)); // CANCoder outputs between (-0.5, 0.5)

        steerController = new PIDController(drivePIDGains.p, drivePIDGains.i, drivePIDGains.d);
        

        configureMotors(steerPIDGains);
    }


    private void configureMotors(PIDGains steerGains) {

        //driveMotor.setSensorPhase(true); // invert the sensor
        driveMotor.setInverted(true); // invert the motor
        driveMotor.setNeutralMode(NeutralModeValue.Brake); // motor brakes when idle


        driveMotor.setSensorPhase(false);
        driveMotor.setInverted(false);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);


        steerController.setP(steerGains.p);
        steerController.setI(steerGains.i);
        steerController.setD(steerGains.d);

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
        
        driveMotor.set(TalonFXControlMode.PercentOutput, (driveOutput + driveFeedforward) / FALCON_500_NOMINAL_VOLTAGE); // Output Voltage / Max Voltage = Percent Output

        double currentAngleMod = MathUtil.angleModulus(getModuleRotation().getRadians());
        double adjustedAngle = optimizedState.angle.getRadians() + getModuleRotation().getRadians() - currentAngleMod;

        if(optimizedState.angle.getRadians() - currentAngleMod > Math.PI) {
            adjustedAngle -= 2.0 * Math.PI;
        }else if(optimizedState.angle.getRadians() - currentAngleMod < -Math.PI) {
            adjustedAngle += 2.0 * Math.PI;
        }
        adjustedDesiredAngle = adjustedAngle;
        steerMotor.set(TalonFXControlMode.Position, adjustedAngle / steerPositionConversionFactor);
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
        return steerMotor.getSelectedSensorPosition() * steerPositionConversionFactor;
    }
    
    private double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * drivePositionConversionFactor;
    }

    private double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * driveVelocityConversionFactor;
    }
}
