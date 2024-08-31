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
    private final DigitalInput beamBreakSensor;
    
    //PID values for intake
    private PIDGains pid = new PIDGains(1, 0, 0, 0, 0, 0, -1, 1);
    

    private boolean pieceLoaded = false;
    private boolean pieceLoadedAtLastCheck = false;
    
    
    public final SysIdRoutine intakeRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
            (voltage) -> setIntakeVelocity(voltage.in(Units.Volts)),
            null,
            this
    ));

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    private boolean pieceDetected = false;
    private double beambreakPasses = 0;
    
    public IntakeSubsystem(){

        //Initialize the motors
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless); //find motor id
        intakeMotor.setInverted(false);

        intakeMotor2 = new CANSparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless); //find motor id
        intakeMotor2.setInverted(true);
        intakeMotor2.follow(intakeMotor, true);

        //Initialize the beam break sensor
        beamBreakSensor = new DigitalInput(INTAKE_BEAM_BREAK_PORT); //find port number
        
        //Initializes encoders
        intakeEncoder = intakeMotor.getEncoder();
        
        //sets encoder positions to 0
        intakeEncoder.setPosition(0);
        
        //initializes intake PID controller to the PID controller in intakeMotor 
        intakePidController = intakeMotor.getPIDController();
        //sets PID values
        Util.setPidController(intakePidController, pid);

    }

    public void teleopInit() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        updateShooterTelemetry();
        filterCurrent();
        countBeambreakPasses();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + intakeMotor.getOutputCurrent() * currentFilterConstant;
    }

    private void countBeambreakPasses() {        
        if (pieceDetected != isPieceDetected()) {
            pieceDetected = isPieceDetected();
            beambreakPasses += 1 * Math.signum(intakeEncoder.getVelocity());
        }
    }
    
    public void updateShooterTelemetry(){
        pieceDetectedPublisher.set(isPieceDetected());
        intakeVelocityPublisher.set(intakeEncoder.getVelocity());
        filteredCurrentPublisher.set(filteredCurrent);
        hasPiecePublisher.set(hasPiece());
        currentCommandPublisher.set(getCurrentCommand() != null? getCurrentCommand().getName() : "null");
    }
    
    /**
     * @return If the beambreak sensor is triggered
     */
    public boolean isPieceDetected(){
        // The sensor returns false when the beam is broken
        return !beamBreakSensor.get();
    }

    /**
     * By counting the number of times the beam break sensor is triggered, we can determine if the turbotake has a piece even if the beam break is not currently triggered.
     * 
     * @return If the turbotake thinks it has a piece
     */
    public boolean hasPiece() {
        return beambreakPasses % 2 == 1 || isPieceDetected();
    }

    /* 
    public double getShooterVelocity() {
        return rightShooterEncoder.getVelocity();
    }*/

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    
    // commands the intake to a target velocity
    public void setIntakeVelocity(double velocity){
         System.out.println("RUNNING INTAKE: " + velocity);
         intakePidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
    public void setIntakePercent(double percent){
        System.out.println("RUNNING INTAKE, SPEED " + percent);
        intakePidController.setReference(percent, ControlType.kDutyCycle);
    }

    //turn off intake and sets Iaccum to 0 to reset I term
    public void turnOffIntake(){
        setIntakePercent(0);
        intakePidController.setIAccum(0);

    }

    public void checkNoteIntake(){
        pieceLoadedAtLastCheck = pieceLoaded;
        pieceLoaded = isPieceDetected();
        if(!pieceLoadedAtLastCheck && pieceLoaded){
            turnOffIntake();
        }
    }


    
    // Command 
    public Command shooterSysIdCommand(){
        return Commands.sequence(
                intakeRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5), 
                intakeRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitCommand(5),
                intakeRoutine.dynamic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5),
                intakeRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }
    


    public Command intake(){
        return Commands.sequence(
            Commands.waitUntil(() -> !isPieceDetected()),
            Commands.runOnce(() -> setIntakePercent(-1)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> turnOffIntake())
        ).finallyDo(() -> {
            turnOffIntake();
        });
    }
}