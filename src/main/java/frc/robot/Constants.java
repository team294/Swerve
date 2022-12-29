// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final class Ports{

        public static final int CANDriveFrontLeftMotor = 1;
        public static final int CANDriveFrontRightMotor = 2;
        public static final int CANDriveBackLeftMotor = 3;
        public static final int CANDriveBackRightMotor = 4;

        public static final int CANDriveTurnFrontLeftMotor = 5;
        public static final int CANDriveTurnFrontRightMotor = 6;
        public static final int CANDriveTurnBackLeftMotor = 7;
        public static final int CANDriveTurnBackRightMotor = 8;

        // Note:  Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
        // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
        // This applies to the turn encoders, which are used as remote sensors for the turn motors.
        public static final int CANTurnEncoderFrontLeft = 9;
        public static final int CANTurnEncoderFrontRight = 10;
        public static final int CANTurnEncoderBackLeft = 11;
        public static final int CANTurnEncoderBackRight = 12;

    }

    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        public static final int usbCoPanel = 3;

        public static final double joystickDeadband = 0.02;
    }

    public static final class RobotDimensions {
        // TODO Update values
        //left to right distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
        //front-back distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

    }

    public static final class SwerveConstants {
        //TODO calibrate this value
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    
        //TODO calibrate these values
        public static final double kEncoderCPR = 2048.0;                // Encoder counts per revolution of FalconFX motor pinion gear
        public static final double kDriveGearRatio = (8.14 / 1.0);      // Team364 = 6.86:1.  Mk4i = 8.14 : 1
        public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // Team364 = 12.8:1.  Mk4i = 150/7 : 1
        public static final double kWheelDiameterMeters = 0.097;        // Depends a little on the tread wear!
        public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
        public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;

        //TODO calibrate this value
        public static final double kPModuleTurningController = 1;
    
        //TODO calibrate this value
        public static final double kPModuleDriveController = 1;
        
        //TODO calibrate this block
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquare = 3.8;
        public static final double kMaxTurningRadiansPerSecond = 12.0;
        public static final double kVDrive = 0.0; //in % output per meters per second
        public static final double kADrive = 0.0;
        public static final double kSDrive = 0.0; //in % output
        public static final double kPDrive = 0.0; //in % output per meters per second
        public static final double kDDrive = 0;
        public static final double kIDrive = 0;
        public static final double kAngularDrive = 0.00;

        public static final double kVTurn = 0.00041;
        public static final double kATurn = 0.0;
        public static final double kSTurn = 0.02;
        public static final double kPTurn = 0.0004;
        public static final double kDTurn = 0.0;
        public static final double kITurn = 0.0;
        public static final double kAngularTurn = 0.0;  //last year's constant of differntiable drive, need to calibrate for swerve drive. above this mark
      }

      public static final class DriveConstants {
        // The locations of the wheels relative to the physical center of the robot, in meters.
        // X: + = forward.  Y: + = to the left
        // The order in which you pass in the wheel locations is the same order that
        // you will receive the module states when performing inverse kinematics. It is also expected that
        // you pass in the module states in the same order when calling the forward kinematics methods.
        // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2));

        // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
        // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
        public static double offsetAngleFrontLeftMotor = 0;
        public static double offsetAngleFrontRightMotor = 0;
        public static double offsetAngleBackLeftMotor = 0;
        public static double offsetAngleBackRightMotor = 0;

        //TODO calibrate this value
        public static final double kMaxSpeedMetersPerSecond = 3;
      }
}

