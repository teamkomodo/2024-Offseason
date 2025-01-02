package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final DoublePublisher intakeCurrentPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("intakeCurrent").publish();
    private final BooleanPublisher pieceIntakedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceIntaked").publish();
    private final BooleanPublisher pieceLoadedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceInShooter").publish();
    private final BooleanPublisher pieceIntakedSensorPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceIntakedSensor").publish();
    private final BooleanPublisher pieceLoadedSensorPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceInShooterSensor").publish();

    private final CANSparkMax intakeMotor;
    private final CANSparkMax intakeMotor2;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePidController;

    public final DigitalInput noteIntakedSensor;
    public final DigitalInput noteLoadedSensor;

    private PIDGains pid = new PIDGains(1, 0, 0, 1, 0, 0, -1, 1);

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    public boolean noteIntaked;
    public boolean noteLoaded;
    
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setInverted(true);

        intakeMotor2 = new CANSparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless);
        intakeMotor2.follow(intakeMotor, true);

        noteIntakedSensor = new DigitalInput(NOTE_INTAKE_BEAM_BREAK_PORT);
        noteLoadedSensor = new DigitalInput(NOTE_LOAD_BEAM_BREAK_PORT);
        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);

        intakePidController = intakeMotor.getPIDController();
        Util.setPidController(intakePidController, pid);
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
        boolean currentNoteIntaked = getNoteDetection(noteIntakedSensor);
        boolean currentNoteLoaded = getNoteDetection(noteLoadedSensor);

        if(!noteIntaked && currentNoteIntaked)
            noteLoaded = true;
        
        if(!noteLoaded && currentNoteLoaded) {
            noteIntaked = true;
            noteLoaded = true;
        }
    }

    public void updateTelemetry() {
        pieceIntakedPublisher.set(noteIntaked);
        pieceLoadedPublisher.set(noteLoaded);
        pieceIntakedSensorPublisher.set(getNoteDetection(noteIntakedSensor));
        pieceLoadedSensorPublisher.set(getNoteDetection(noteLoadedSensor));
        intakeCurrentPublisher.set(intakeMotor.getOutputCurrent());
    }
    
    public void setIntakeDutyCycle(double dutyCycle) {
        intakePidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public Command eject() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setIntakeDutyCycle(-0.5)),
            Commands.waitSeconds(0.7),
            Commands.runOnce(() -> holdIntake()),
            Commands.runOnce(() -> {noteIntaked = false;}),
            Commands.runOnce(() -> {noteLoaded = false;})
        );
    }

    public void holdIntake() {
        intakePidController.setReference(intakeEncoder.getPosition(), ControlType.kPosition);
    }

    private boolean getNoteDetection(DigitalInput beamBreak) {
        return beamBreak.get();
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    public boolean getNoteIntaked() {
        return noteIntaked;
    }

    public boolean getNoteLoaded() {
        return noteLoaded;
    }
}