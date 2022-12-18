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
    }

    public static final class RobotDimensions {
        // TODO Update values
        //left to right distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
        //front-back distance between the drivetrain wheels; should be measured from center to center
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

    }

    public static final class ModuleConstants {
        //TODO calibrate this value
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    
        //TODO calibrate this value
        public static final int kEncoderCPR = 1024;                 // Encoder counts per revolution
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderMetersPerTick = 1;
            // Assumes the encoders are directly mounted on the wheel shafts
            //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        
        //TODO calibrate this value
        public static final double kPModuleTurningController = 1;
    
        //TODO calibrate this value
        public static final double kPModuleDriveController = 1;
        
        public static final double kMaxSpeedMetersPerSecond = 5.22; //last year's constant of differntiable drive, need to calibrate for swerve drive. below this mark
        public static final double kMaxAccelerationMetersPerSecondSquare = 3.8;
        public static final double compensationVoltage = 12.0;
        public static final double kVDrive = 0.187;
        public static final double kADrive = 0.025;
        public static final double kSDrive = 0.024;
        public static final double kPDrive = 0.280;
        public static final double kDDrive = 0;
        public static final double kIDrive = 0;
        public static final double kAngularDrive = 0.030;



        public static final double kVTurn = 1.0;
        public static final double kATurn = 1.0;
        public static final double kSTurn = 1.0;
        public static final double kPTurn = 1.0;
        public static final double kDTurn = 1.0;
        public static final double kITurn = 1.0;
        public static final double kAngularTurn = 1.0;  //last year's constant of differntiable drive, need to calibrate for swerve drive. above this mark
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

        //TODO add 4 offset angle (0 for now, calibrate later)
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

