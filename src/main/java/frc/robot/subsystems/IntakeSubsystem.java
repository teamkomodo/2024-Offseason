package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private final BooleanPublisher pieceIntakedSensorPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceIntakedSensor").publish();
    private final BooleanPublisher pieceLoadedSensorPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("pieceInShooterSensor").publish();

    private final CANSparkMax intakeMotor;
    private final CANSparkMax intakeMotor2;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePidController;

    private final DigitalInput noteIntakedSensor;
    private final DigitalInput noteLoadedSensor;

    public String currentCommand = "idle";

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

    public boolean noteIntaked;
    public boolean noteLoaded;
    
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeMotor.setSmartCurrentLimit(90);
        intakeMotor.setInverted(true);

        intakeMotor2 = new CANSparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeMotor2.setInverted(false);
        intakeMotor2.follow(intakeMotor, true);

        noteIntakedSensor = new DigitalInput(1); //FIXME: find port number
        noteLoadedSensor = new DigitalInput(2); //FIXME: find port number
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

        if(currentNoteIntaked && getIntakeVelocity() > 0) {
            noteIntaked = true;
        }

        if(currentNoteLoaded && getIntakeVelocity() > 0 && currentCommand == "intake") {
            holdIntake();
            noteIntaked = true;
            noteLoaded = true;
        }
        
        if(!currentNoteIntaked && currentCommand == "eject") {
            noteLoaded = false;
            noteIntaked = false;
        }
    }

    public void updateTelemetry() {
        pieceIntakedPublisher.set(getPieceIntaked());
        pieceLoadedPublisher.set(getPieceLoaded());
        pieceIntakedSensorPublisher.set(getNoteDetection(noteIntakedSensor));
        pieceLoadedSensorPublisher.set(getNoteDetection(noteLoadedSensor));
        intakeVelocityPublisher.set(intakeEncoder.getVelocity());
    }

    public void setIntakeVelocity(double velocity) {
         setMotor(velocity, ControlType.kVelocity);
    }
    
    public void setIntakeDutyCycle(double dutyCycle) {
        intakePidController.setReference(dutyCycle, ControlType.kDutyCycle);
        intakeDutyCycle = dutyCycle;
    }

    public Command intake() {
        if(!noteLoaded) {
            currentCommand = "intake";
            return Commands.runEnd(() -> setIntakeDutyCycle(0.8), () -> {setIntakeDutyCycle(0); noteLoaded = true; noteIntaked = true;}).until(() -> getNoteDetection(noteLoadedSensor));
        } else {
            return Commands.runOnce(() -> holdIntake());
        }
    }

    public Command eject() {
        currentCommand = "eject";
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setIntakeDutyCycle(-0.5)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> holdIntake()),
            Commands.runOnce(() -> {noteIntaked = false;}),
            Commands.runOnce(() -> {noteLoaded = false;})
        );
    }

    public void holdIntake() {
        currentCommand = "idle";
        setMotor(intakeEncoder.getPosition(), ControlType.kPosition);
    }

    private void setMotor(double value, ControlType type) {
        if (type == ControlType.kDutyCycle)
            intakeDutyCycle = value;
        intakePidController.setReference(value, type);
    }

    public boolean getNoteDetection(DigitalInput beamBreak) {
        return beamBreak.get();
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