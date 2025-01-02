// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean TUNING_MODE = false;

    // Controls
    public static final double XBOX_DEADBAND = 0.06;
    public static final int DRIVER_XBOX_PORT = 0;
    public static final int OPERATOR_XBOX_PORT = 1;

    // Shooter
    // //Shooter
    public static final int SHOOTER_MOTOR_1_ID = 25;
    public static final int SHOOTER_MOTOR_2_ID = 26;

    //Intake
    public static final int INTAKE_MOTOR_1_ID = 21;
    public static final int INTAKE_MOTOR_2_ID = 22;
    public static final int NOTE_LOAD_BEAM_BREAK_PORT = 2; 
    public static final int NOTE_INTAKE_BEAM_BREAK_PORT = 1; 
    public static final double INTAKE_SPEED = -0.5;

    // Joint
    public static final int JOINT_MOTOR_1_ID = 23; 
    public static final int JOINT_MOTOR_2_ID = 24;
    public static final int JOINT_ZERO_SWITCH_CHANNEL = 0;

    public static final double JOINT_STOW_POSITION = 0.0;
    public static final double JOINT_SHOOTING_POSITION = 9.367;
    public static final double JOINT_MIN_POSITION = 0.0;
    public static final double JOINT_MAX_POSITION = 9.367;

    // Constants

    // rpm
    public static final double SPEAKER_SPEED = 3000;
    public static final double SPIN_RATIO = 0.3;

	public static final boolean FIELD_RELATIVE_DRIVE = true;
	public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
	public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.2;
	
	public static final double DRIVETRAIN_WIDTH = 0.4864163246;
	public static final double DRIVETRAIN_LENGTH = 0.4079583662;
	
	public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
	public static final int FRONT_LEFT_STEER_MOTOR_ID = 5;
	public static final int FRONT_LEFT_STEER_ENCODER_ID = 10;
	public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(293.1+180)+1.346;
	
	public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
	public static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
	public static final int FRONT_RIGHT_STEER_ENCODER_ID = 11;
	public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(357.78)+1.422;
	
	public static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
	public static final int BACK_LEFT_STEER_MOTOR_ID = 3;
	public static final int BACK_LEFT_STEER_ENCODER_ID = 9;
	public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(327.3)+9.235;
	
	public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0;
	public static final int BACK_RIGHT_STEER_MOTOR_ID = 1;
	public static final int BACK_RIGHT_STEER_ENCODER_ID = 8;
	public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(170.75)+9.259;
	
	public static final double WHEEL_DIAMETER = 0.1016;
	public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
	public static final double STEER_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
	
	public static final double MAX_MODULE_VELOCITY = 4.058; // physical maximum attainable speed of swerve modules
	public static final double MAX_MODULE_ACCEL = 21; // physical maximum attainable accel of swerve modules
	
	public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI; // constraint for angular velocity
	public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI; // constraint for angular acceleration
	
	public static final double FALCON_500_NOMINAL_VOLTAGE = 12.0;
	public static final double TALON_FX_TICKS_PER_ROTATION = 2048.0;

	public static HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
			new PIDConstants(5.0, 0.0, 0.0), // Translation Constants
			new PIDConstants(5.0, 0.0, 0.0), // Steering Constants
			MAX_MODULE_VELOCITY,
			Math.sqrt(DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) / 2,
			new ReplanningConfig(false, false)
	);

    public static final BooleanSupplier ON_RED_ALLIANCE = () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            };

    // FRC Field
    public static final double FIELD_WIDTH = 8.21; // m approxiamation: Field Length is 26ft. 11 1/8 in wide
    public static final double FIELD_LENGTH = 16.54;

    public static final int A_FRAME_LED_CHANNEL = 0;
    public static final int TURBOTAKE_LED_CHANNEL = 1;





    


}
