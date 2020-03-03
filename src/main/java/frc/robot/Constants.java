/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Control panel motor
    public static final int CONTROL_PANEL_MOTOR_PORT = 9;
    public static final double CP_WHEEL_SPEED = .125;

    // Color sensor constants
    public static final Color BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color GREEN_TARGET = ColorMatch.makeColor(0.170, 0.561, 0.240);
    public static final Color RED_TARGET = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color YELLOW_TARGET = ColorMatch.makeColor(0.361, 0.524, 0.113);

    //Shooter Constants
    public static final double S_BACKSPIN_RATIO = 4 / 7;
    public static final double S_WHEEL_SPEED = 3600 / 60; //rps
    public static final double S_WHEEL_VOLTAGE = 10; //Measure pls
    public static final double S_PROPORTIONAL_CONSTANT = 6;
    public static final double S_INTEGRAL_CONSTANT = 5;
    public static final double S_DERIVATIVE_CONSTANT = 4;
    public static final int SHOOTER_TOP_MOTOR = 5;
    public static final int SHOOTER_BOTTOM_MOTOR = 6;
    public static final double AUTON_SHOOTER_SPEED = .7;
    public static final int SHOOTER_PIVOT_MOTOR = 10; //NEEDS TO BE ADDED STILL

    public static final double ROBOT_HEIGHT = 32.222 / 12;
    public static final double WHEEL_CIRCUMFERENCE = 5 * Math.PI / 12;
    public static final double SHOOTER_MOUTH_WIDTH = 10; //Currently a placeholder best be in feet
    public static final int GIMBAL_LOCK_PORT2 = 6; //find
    public static final int GIMBAL_LOCK_PORT1 = 9; //do
    public static final double GIMBAL_MULTIPLIER = 5; //be
    public static final int HOOD_GEAR_RATIO = 18 / 26;

    //Drive train stuff
    public static final int LEFT_DRIVE_MOTOR_1 = 1;
    public static final int LEFT_DRIVE_MOTOR_2 = 2;
    public static final int LEFT_DRIVE_MOTOR_3 = 3;
    public static final int RIGHT_DRIVE_MOTOR_1 = 4;
    public static final int RIGHT_DRIVE_MOTOR_2 = 5;
    public static final int RIGHT_DRIVE_MOTOR_3 = 6;

    public static final double DRIVE_CONVERSION_FACTOR = 0;

    //intake
    public static final int INTAKE_MOTOR_PORT = 7;

    //conveyor
    public static final int CONVEYOR_MOTOR = 8;
    public static final int BALL_SENSOR_PORT = 0;
    public static final int EMPTY_SENSOR_PORT = 5; //Check this one
    public static final double AUTON_CONVEYOR_SPEED = .5;

    //auton constants
    //haha dont touch os
    public static final double ksVolts = .185;
    public static final double kvVoltSecondsPerMeter = 2.11;
    public static final double kaVoltSecondsSquaredPerMeter = .571;
    public static final double kPDriveVel = 1.82;
    public static final double kTrackwidthMeters = .64557;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(1);
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(21);
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //vision auto constants
    public static final double TOLERANCE = 10;
    public static final double SPEED_INDEX = .001;
    public static final double BALL_AREA = 1;
    public static final double BALL_DISTANCE = 1;
    public static final double GOAL_AREA = .00001;
    public static final double GOAL_DISTANCE = 1;

    //Command IDs for Driver Input
    //First Digit = Main Subsystem Number
    //1 = DriveTrain
    //2 = Conveyor
    //3 = BallIntake
    //4 = EVS
    //Second Digit = Command ID
    public static final int DRIVE_MAX_SPEED_ID = 12;
    public static final int DRIVE_NORMAL_ID = 11;
    public static final int DRIVE_SLOW_SPEED_ID = 13;
    public static final int RUN_INTAKE_ID = 31;
    public static final int RUN_INTAKE_BACKWARDS_ID = 32;
    public static final int STOP_INTAKE_ID = 33;
    public static final int TOGGLE_VISION_ID = 41;
    public static final int AIM_AND_SHOOT_ID = 42;

    /*
    [Name Low-Speed High Speed]
    
    */

    //public static String[][] driverInputArray = new String{{"Christian Piper", "0.25", "0.75"}, {"Standard", "0",}};

}
