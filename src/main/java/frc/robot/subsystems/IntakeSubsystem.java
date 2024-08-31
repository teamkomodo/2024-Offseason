package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase{
    //Telemetry
    
    //motor telemetry
    private final DoublePublisher intakeVelocityPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("intakevelocity").publish();
    private final DoublePublisher filteredCurrentPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("filteredcurrent").publish();

    //beam break sensor telemetry
    private final BooleanPublisher pieceDetectedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("piecedetected").publish();
    private final BooleanPublisher hasPiecePublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("haspiece").publish();

    private final StringPublisher currentCommandPublisher = NetworkTableInstance.getDefault().getTable("intake").getStringTopic("currentcommand").publish();

    // dupe motors

    //defines motors
    private final CANSparkMax intakeMotor;
    private final CANSparkMax intakeMotor2;
    
    //Encoders
    private final RelativeEncoder intakeEncoder;
    
    //PID Controllers
    private final SparkPIDController intakePidController;
    
    //defines beam break sensor
    private final DigitalInput noteIntakedSensor;
    private final DigitalInput noteBeforeShooterSensor;
    
    //PID values for intake
    private PIDGains pid = new PIDGains(1, 0, 0, 1, 0, 0, -1, 1);
    
    public final SysIdRoutine intakeRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
            (voltage) -> setIntakeVelocity(voltage.in(Units.Volts)),
            null,
            this
    ));

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    private boolean noteWasIntaked = false;
    private boolean noteWasBeforeShooter = false;
    private boolean intakeHasPiece = false;
    private boolean pieceAtShooter = false;
    
    public IntakeSubsystem(){

        //Initialize the motors
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeMotor.setInverted(false);

        intakeMotor2 = new CANSparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless); //FIXME: find motor id
        intakeMotor2.setInverted(true);
        intakeMotor2.follow(intakeMotor, true);

        //Initialize the beam break sensor
        noteIntakedSensor = new DigitalInput(INTAKE_BEAM_BREAK_PORT); //FIXME: find port number
        noteBeforeShooterSensor = new DigitalInput(SHOOTER_BEAM_BREAK_PORT); //FIXME: find port number
        
        //Initializes encoders
        intakeEncoder = intakeMotor.getEncoder();
        
        //sets encoder positions to 0
        intakeEncoder.setPosition(0);
        
        //initialize intake PID controller and set PID values
        intakePidController = intakeMotor.getPIDController();
        Util.setPidController(intakePidController, pid);

        // Set sensor prev values
        noteWasIntaked = noteIntakedSensor.get();
        noteWasBeforeShooter = noteBeforeShooterSensor.get();
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
        if (sensorSeesPiece(noteBeforeShooterSensor)) {
            pieceAtShooter = true;
            turnOffIntake();
        } else {
            pieceAtShooter = false;
        }
        
        intakeHasPiece = sensorSeesPiece(noteIntakedSensor);

        noteWasIntaked = sensorSeesPiece(noteIntakedSensor);
        noteWasBeforeShooter = sensorSeesPiece(noteBeforeShooterSensor);
    }

    public void updateTelemetry(){
        pieceDetectedPublisher.set(hasPieceReady());
        intakeVelocityPublisher.set(intakeEncoder.getVelocity());
        filteredCurrentPublisher.set(filteredCurrent);
        hasPiecePublisher.set(hasPieceReady());
        currentCommandPublisher.set(getCurrentCommand() != null? getCurrentCommand().getName() : "null");
    }
    
    /**
     * @return If the beambreak sensor sees a note
     */
    public boolean sensorSeesPiece(DigitalInput sensor){
        // The sensor returns false when the beam is broken
        return !sensor.get();
    }

    /**
     * @return If the turbotake thinks it has a piece
     */
    public boolean hasPieceReady() {
        return pieceAtShooter;
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    // Command the intake to a target velocity
    public void setIntakeVelocity(double velocity){
         System.out.println("RUNNING INTAKE: " + velocity);
         intakePidController.setReference(velocity, ControlType.kVelocity);
    }
    
    public void setIntakeDutyCycle(double dutyCycle){
        System.out.println("RUNNING INTAKE, SPEED " + dutyCycle);
        intakePidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    // Turn off intake and set Iaccum to 0 to reset I term
    public void turnOffIntake(){
        setIntakeDutyCycle(0);
        intakePidController.setIAccum(0);
    }

    public Command intake(){
        return Commands.sequence(
            Commands.waitUntil(() -> (!hasPieceReady())),
            Commands.runOnce(() -> setIntakeDutyCycle(INTAKE_SPEED)),
            Commands.waitUntil(() -> hasPieceReady()),
            Commands.waitSeconds(0.005),
            Commands.runOnce(() -> turnOffIntake())
        ).finallyDo(() -> {
            turnOffIntake();
        });
    }
}