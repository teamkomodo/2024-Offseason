package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDGains;
import frc.robot.util.Util;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private final DoublePublisher intakeVelocityPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("intakeDutyCycle").publish();
    private final BooleanPublisher pieceIntakedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceIntaked").publish();
    private final BooleanPublisher pieceLoadedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceInShooter").publish();

    private final CANSparkMax intakeMotor;
    private final CANSparkMax intakeMotor2;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePidController;

    private final DigitalInput noteIntakedSensor;
    private final DigitalInput noteLoadedSensor;

    private PIDGains pid = new PIDGains(1, 0, 0, 1, 0, 0, -1, 1);
    
    // public final SysIdRoutine intakeRoutine = new SysIdRoutine(
    //         new SysIdRoutine.Config(), 
    //         new SysIdRoutine.Mechanism(
    //         (voltage) -> setIntakeVelocity(voltage.in(Units.Volts)),
    //         null,
    //         this
    // ));

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;
    private double intakeDutyCycle;

    private boolean noteIntaked;
    private boolean noteIntakedAtLastCheck;
    private boolean noteLoaded;
    private boolean noteLoadedAtLastCheck;
    
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setInverted(false);

        intakeMotor2 = new CANSparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeMotor2.setInverted(true);
        intakeMotor2.follow(intakeMotor, true);

        noteIntakedSensor = new DigitalInput(INTAKE_BEAM_BREAK_PORT); //FIXME: find port number
        noteLoadedSensor = new DigitalInput(SHOOTER_BEAM_BREAK_PORT); //FIXME: find port number

        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);

        intakePidController = intakeMotor.getPIDController();
        Util.setPidController(intakePidController, pid);

        noteIntaked = getNoteDetection(noteIntakedSensor);
        noteIntakedAtLastCheck = false;
        noteLoaded = getNoteDetection(noteLoadedSensor);
        noteLoadedAtLastCheck = false;

    }

    public void teleopInit() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        updateTelemetry();
        filterCurrent();
        checkSensors();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + intakeMotor.getOutputCurrent() * currentFilterConstant;
    }

    public void checkSensors() {
        noteIntaked = getNoteDetection(noteIntakedSensor);
        noteLoaded = getNoteDetection(noteLoadedSensor);

        if(noteIntaked && !noteIntakedAtLastCheck && intakeEncoder.getVelocity() > 0) {
            //LED blinkers and controller vibrations
        }
        if(noteLoaded && !noteLoadedAtLastCheck)
            holdIntake();
        noteLoadedAtLastCheck = noteLoaded;
        noteIntakedAtLastCheck = noteIntaked;
    }

    public boolean checkEjectStatus() {
        return !noteIntaked && noteIntakedAtLastCheck && intakeEncoder.getVelocity() < 0;
    }

    public void updateTelemetry() {
        pieceIntakedPublisher.set(noteIntaked);
        pieceLoadedPublisher.set(noteLoaded);
        intakeVelocityPublisher.set(intakeDutyCycle);
    }

    public void setIntakeVelocity(double velocity) {
         setMotor(velocity, ControlType.kVelocity);
    }
    
    public void setIntakeDutyCycle(double dutyCycle) {
        setMotor(dutyCycle, ControlType.kDutyCycle);
    }

    public Command intake() {
        return this.runOnce(() -> setIntakeDutyCycle(0.1));
    }

    public Command eject() {
        return Commands.runEnd(() -> setIntakeDutyCycle(-0.1), () -> setIntakeDutyCycle(0), this).until(() -> (checkEjectStatus()));
    }

    public void holdIntake() {
        setMotor(intakeEncoder.getPosition(), ControlType.kPosition);
    }

    private void setMotor(double value, ControlType type) {
        if (type == ControlType.kDutyCycle)
            intakeDutyCycle = value;
        intakePidController.setReference(value, type);
    }

    // public Command intake() {
    //     return Commands.sequence(
    //         Commands.waitUntil(() -> (!getHasPieceReady())),
    //         Commands.runOnce(() -> setIntakeDutyCycle(INTAKE_SPEED)),
    //         Commands.waitUntil(() -> getHasPieceReady()),
    //         Commands.waitSeconds(0.005),
    //         Commands.runOnce(() -> turnOffIntake())
    //     ).finallyDo(() -> {
    //         turnOffIntake();
    //     });
    // }

    public boolean getNoteDetection(DigitalInput beamBreak) {
        return !beamBreak.get();
    }

    public boolean getPieceIntaked() {
        return noteIntaked;
    }

    public boolean getPieceLoaded() {
        return noteLoaded;
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }
}