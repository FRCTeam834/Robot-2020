/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units

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
    public static final double S_BOTTOM_WHEEL_SPEED = 3500;
    public static final double S_PROPORTIONAL_CONSTANT = 6;
    public static final double S_INTEGRAL_CONSTANT = 5;
    public static final double S_DERIVATIVE_CONSTANT = 4;
    public static final int SHOOTER_TOP_MOTOR = 6;
    public static final int SHOOTER_BOTTOM_MOTOR = 5;

    //Drive train stuff
    public static final int L_DRIVE1_PORT = 1;
    public static final int L_DRIVE2_PORT = 2;
    public static final int R_DRIVE1_PORT = 3;
    public static final int R_DRIVE2_PORT = 4;

    //intake
    public static final int INTAKE_MOTOR_PORT = 7;

    //conveyor
    public static final int CONVEYOR_MOTOR = 8;
    public static final int BALL_SENSOR_PORT = 0;

    //auton constants
    //haha dont touch os
    public static final double ksVolts = .146;
    public static final double kvVoltSecondsPerMeter = 2.18;
    public static final double kaVoltSecondsSquaredPerMeter = .571;
    public static final double kPDriveVel = 18.2;
    public static final double kTrackwidthMeters = .629796255;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(3);
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(5);
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}
