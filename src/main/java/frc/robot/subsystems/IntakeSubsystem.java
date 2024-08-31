package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Units;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase{

    // dupe motors

    //defines motors
    private final CANSparkMax intakeMotorLeft;
    private final CANSparkMax intakeMotorRight;
    
    //Encoders
    private final RelativeEncoder intakeEncoderLeft;
    private final RelativeEncoder intakeEncoderRight;
    
    //PID Controllers
    private final SparkPIDController intakePidControllerLeft;
    private final SparkPIDController intakePidControllerRight;
    
    //defines beam break sensor
    private final DigitalInput beamBreakSensor;
    
    //Telemetry
    
    //motor telemetry
    private final DoublePublisher intakeVelocityPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("intakevelocity").publish();
    private final DoublePublisher filteredCurrentPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("filteredcurrent").publish();

    //beam break sensor telemetry
    private final BooleanPublisher pieceDetectedPublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("piecedetected").publish();
    private final BooleanPublisher hasPiecePublisher = NetworkTableInstance.getDefault().getTable("intake").getBooleanTopic("haspiece").publish();

    private final StringPublisher currentCommandPublisher = NetworkTableInstance.getDefault().getTable("intake").getStringTopic("currentcommand").publish();

    private final DoubleEntry intakePEntry, intakeIEntry, intakeDEntry;

    //PID values for intake
    private double intakeP, intakeI, intakeD, intakeIZone, intakeFF, intakeMinOutput, intakeMaxOutput;
    

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

        //PID coefficients for intake
        intakeP = 1;
        intakeI = 0;
        intakeD = 0;
        intakeIZone = 0;
        intakeFF = 0;
        intakeMinOutput = -1;
        intakeMaxOutput = 1;

        intakePEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/intakekP").getEntry(intakeP);
        intakeIEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/intakekI").getEntry(intakeI);
        intakeDEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/intakekD").getEntry(intakeD);

        intakePEntry.set(intakeP);
        intakeIEntry.set(intakeI);
        intakeDEntry.set(intakeD);

        //Initialize the motors
        intakeMotorLeft = new CANSparkMax(INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless); //find motor id left
        intakeMotorRight = new CANSparkMax(INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless); //find motor id right
        
        //inverts motors to correct orientation
        intakeMotorLeft.setInverted(false); //find out
        intakeMotorRight.setInverted(false); //find out

        //Initialize the beam break sensor
        beamBreakSensor = new DigitalInput(INTAKE_BEAM_BREAK_PORT); //find port number
        
        //Initializes encoders
        intakeEncoderLeft = intakeMotorLeft.getEncoder();
        intakeEncoderRight = intakeMotorRight.getEncoder();
        
        //sets encoder positions to 0
        intakeEncoderLeft.setPosition(0);
        intakeEncoderRight.setPosition(0);
        
        //initializes intake PID controller to the PID controller in intakeMotor Left
        intakePidControllerLeft = intakeMotorLeft.getPIDController();
        //sets PID values Left
        intakePidControllerLeft.setP(intakeP);
        intakePidControllerLeft.setI(intakeI);
        intakePidControllerLeft.setD(intakeD);
        intakePidControllerLeft.setIZone(intakeIZone);
        intakePidControllerLeft.setFF(intakeFF);
        intakePidControllerLeft.setOutputRange(intakeMinOutput, intakeMaxOutput);

        //initializes intake PID controller to the PID controller in intakeMotor Left
        intakePidControllerRight = intakeMotorRight.getPIDController();
        //sets PID values Left
        intakePidControllerRight.setP(intakeP);
        intakePidControllerRight.setI(intakeI);
        intakePidControllerRight.setD(intakeD);
        intakePidControllerRight.setIZone(intakeIZone);
        intakePidControllerRight.setFF(intakeFF);
        intakePidControllerRight.setOutputRange(intakeMinOutput, intakeMaxOutput);
        
    }

    public void teleopInit() {
        intakeMotorLeft.set(0);
        intakeMotorRight.set(0);
    }

    @Override
    public void periodic() {
        updateShooterTelemetry();
        filterCurrent();
        countBeambreakPasses();
        updateControlConstants();
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

    public void updateControlConstants() {

        if(!TUNING_MODE)
            return;

        double newIntakeP = intakePEntry.get(intakeP);
        if(newIntakeP != intakeP) {
            intakeP = newIntakeP;
            intakePidControllerLeft.setP(intakeP);
            intakePidControllerRight.setP(intakeP);
        }

        double newIntakeI = intakeIEntry.get(intakeI);
        if(newIntakeI != intakeI) {
            intakeI = newIntakeI;
            intakePidControllerLeft.setI(intakeI);
            intakePidControllerRight.setI(intakeI);
        }

        double newIntakeD = intakeDEntry.get(intakeD);
        if(newIntakeD != intakeD) {
            intakeD = newIntakeD;
            intakePidControllerLeft.setD(intakeD);
            intakePidControllerRight.setD(intakeD);
        }

    

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
        return intakeEncoderLeft.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    
    // commands the intake to a target velocity
    public void setIntakeVelocity(double velocity){
         System.out.println("RUNNING INTAKE: " + velocity);
         intakePidControllerLeft.setReference(velocity, CANSparkMax.ControlType.kVelocity);
         intakePidControllerRight.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
    public void setIntakePercent(double percent){
        System.out.println("RUNNING INTAKE, SPEED " + percent);
        intakePidControllerLeft.setReference(percent, ControlType.kDutyCycle);
        intakePidControllerRight.setReference(percent, ControlType.kDutyCycle);
    }

    //turn off intake and sets Iaccum to 0 to reset I term
    public void turnOffIntake(){
        setIntakePercent(0);
        intakePidControllerLeft.setIAccum(0);
        intakePidControllerRight.setIAccum(0);

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