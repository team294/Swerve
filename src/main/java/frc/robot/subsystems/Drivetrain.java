// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.utilities.*;
import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrain extends SubsystemBase {
  
  //motor modules
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;


  private final WPI_TalonFX leftMotor1;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX rightMotor1;
  private final WPI_TalonFX rightMotor2;

  private final DifferentialDrive diffDrive;
  private final DifferentialDriveOdometry odometry;

  private double leftEncoderZero = 0;
  private double rightEncoderZero = 0;

  private final AHRS ahrs;
  private double yawZero = 0;

  private FileLog log;
  private TemperatureCheck tempCheck;
  
  // variables to help calculate angular velocity for turnGyro
  private double prevAng; // last recorded gyro angle
  private double currAng; // current recorded gyro angle
  private double prevTime; // last time gyro angle was recorded
  private double currTime; // current time gyro angle is being recorded
  private double angularVelocity;  // Robot angular velocity in degrees per second
  private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc


/** Creates a new Drivetrain. */
  public DriveTrain(FileLog log, TemperatureCheck tempCheck) {
    this.log = log; // save reference to the fileLog
    this.tempCheck = tempCheck;

    // configure navX
    AHRS gyro = null;
    try {
      if (prototypeBot) {
        gyro = new AHRS(I2C.Port.kMXP);
      } else {
        gyro = new AHRS(SerialPort.Port.kUSB);
      }
      // gyro.zeroYaw();   // *** Do not zero the gyro hardware!  The hardware zeros asynchronously from this thread, so an immediate read-back of the gyro may not yet be zeroed.
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    ahrs = gyro;

    // FIXME Setup motor configuration
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    ); 
    
    setVoltageCompensation(true);
    setOpenLoopRampLimit(true);

    // create the differential drive AFTER configuring the motors
    diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);
    //diffDrive.setRightSideInverted(true);
    
    diffDrive.setDeadband(0.0);
    
    zeroLeftEncoder();
    zeroRightEncoder();
    zeroGyroRotation();

    // Sets initial position to (0,0) facing 0 degrees
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroRotation()));

    // initialize angular velocity variables
    prevAng = getGyroRaw();
    currAng = getGyroRaw();
    prevTime = System.currentTimeMillis();
    currTime = System.currentTimeMillis();
    lfRunningAvg.reset();

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Drive kV Linear", kVLinear); // Linear coefficients
    SmartDashboard.putNumber("Drive kA Linear", kALinear);
    SmartDashboard.putNumber("Drive kS Linear", kSLinear);
    SmartDashboard.putNumber("Drive kP Linear", kPLinear);
    SmartDashboard.putNumber("Drive kI Linear", kILinear);
    SmartDashboard.putNumber("Drive kD Linear", kDLinear);
    SmartDashboard.putNumber("Drive kAng Linear", kAngLinear);

    SmartDashboard.putNumber("Drive kV Angular", kVAngular); // Angular coefficients
    SmartDashboard.putNumber("Drive kA Angular", kAAngular);
    SmartDashboard.putNumber("Drive kS Angular", kSAngular);
    SmartDashboard.putNumber("Drive kP Angular", kPAngular);
    SmartDashboard.putNumber("Drive kI Angular", kIAngular);
    SmartDashboard.putNumber("Drive kD Angular", kDAngular);
    SmartDashboard.putNumber("Drive tLag Angular", tLagAngular);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
