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
    public static final int INTAKE_MOTOR_1_ID = 21; // FIXME: need proper defenition
    public static final int INTAKE_MOTOR_2_ID = 22; // FIXME: need proper defenition
    public static final int INTAKE_BEAM_BREAK_PORT = 2; // FIXME: need proper defenition
    public static final int SHOOTER_BEAM_BREAK_PORT = 1; // FIXME: need proper defenition
    public static final double INTAKE_SPEED = -0.5;

    // Joint
    public static final int JOINT_MOTOR_1_ID = 23; // FIXME: Change motor ID
    public static final int JOINT_MOTOR_2_ID = 24; // FIXME: Change motor ID
    public static final int JOINT_ZERO_SWITCH_CHANNEL = 0;

    public static final double JOINT_STOW_POSITION = 0.0;
    public static final double JOINT_SHOOTING_POSITION = 1.0;
    public static final double JOINT_MIN_POSITION = 0.0;
    public static final double JOINT_MAX_POSITION = 0.0;

    // Constants

    // rpm
    public static final double SPEAKER_SPEED = 3000;
    public static final double SPIN_RATIO = 0.3;

    public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
    public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.3;
    public static final double DRIVETRAIN_WIDTH = 0.5271; // Distance between center of left and right swerve wheels in meters
    public static final double DRIVETRAIN_LENGTH = 0.5271; // Distance between center of front and back swerve wheels in meters

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 1;
    public static final int BACK_RIGHT_STEER_ENCODER_ID = 8;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(168.65);

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 3;
    public static final int BACK_LEFT_STEER_ENCODER_ID = 9;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(327.3);

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
    public static final int FRONT_RIGHT_STEER_ENCODER_ID = 11;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(356.48);

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 5;
    public static final int FRONT_LEFT_STEER_ENCODER_ID = 10;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(294.1);

    public static final double WHEEL_DIAMETER = 0.1016;

    /**
     * motor rotations -> wheel rotations
     */
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);

    /**
     * motor rotations -> module rotations
     */
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    public static final double MAX_ATTAINABLE_VELOCITY = 3.8;
    public static final double MAX_LINEAR_VELOCITY = 0;

    public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
    public static final double LINEAR_ACCEL_CONSTRAINT = 12.0;

    public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.8;
    public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH);

    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(2, 0, 0),
        new PIDConstants(2, 0, 0),
        MAX_ATTAINABLE_VELOCITY,
        Math.sqrt(DRIVETRAIN_LENGTH*DRIVETRAIN_LENGTH + DRIVETRAIN_WIDTH*DRIVETRAIN_WIDTH)/2,
        new ReplanningConfig()
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
