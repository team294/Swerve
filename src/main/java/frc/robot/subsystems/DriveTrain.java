// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.FileLog;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.utilities.*;


public class DriveTrain extends SubsystemBase implements Loggable {

  // File logging variables
  private FileLog log;
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles

  // create swerve modules
  private final SwerveModule swerveFrontLeft = new SwerveModule(
      CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANTurnEncoderFrontLeft, 
      false, false, offsetAngleFrontLeftMotor);
  private final SwerveModule swerveFrontRight = new SwerveModule(
      CANDriveFrontRightMotor, CANDriveTurnFrontRightMotor, CANTurnEncoderFrontRight, 
      false, false, offsetAngleFrontRightMotor);
  private final SwerveModule swerveBackLeft = new SwerveModule(
      CANDriveBackLeftMotor, CANDriveTurnBackLeftMotor, CANTurnEncoderBackLeft, 
      false, false, offsetAngleBackLeftMotor);
  private final SwerveModule swerveBackRight = new SwerveModule(
      CANDriveBackRightMotor, CANDriveTurnBackRightMotor, CANTurnEncoderBackRight, 
      false, false, offsetAngleBackRightMotor);
  
  // variables for gyro and gyro calibration
  private final AHRS ahrs;
  private double yawZero = 0;

  // variables to help calculate angular velocity for turnGyro
  private double prevAng; // last recorded gyro angle
  private double currAng; // current recorded gyro angle
  private double prevTime; // last time gyro angle was recorded
  private double currTime; // current time gyro angle is being recorded
  private double angularVelocity;  // Robot angular velocity in degrees per second
  private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry;

  /**
   * Constructs the DriveTrain subsystem
   * @param log FileLog object for logging
   */
  public DriveTrain(FileLog log) {
    this.log = log; // save reference to the fileLog

    // configure navX gyro
    AHRS gyro = null;
		try {
      gyro = new AHRS(SerialPort.Port.kUSB);
      // gyro.zeroYaw();   // *** Do not zero the gyro hardware!  The hardware zeros asynchronously from this thread, so an immediate read-back of the gyro may not yet be zeroed.
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    ahrs = gyro;

    // zero gyro and initialize angular velocity variables
    zeroGyroRotation();
    prevAng = getGyroRaw();
    currAng = getGyroRaw();
    prevTime = System.currentTimeMillis();
    currTime = System.currentTimeMillis();
    lfRunningAvg.reset();

    //TODO config swerve modules


    // create and initialize odometery
    odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(getGyroRotation()), 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)) );
  }
  

  // ************ Gryo methods

  /**
   * Verifies if Gyro is still reading
   * @return true = gryo is connected to Rio
   */
  public boolean isGyroReading() {
    return ahrs.isConnected();
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * Angle is negated from the gyro, so that + = left and - = right
   * @return raw gyro angle, in degrees.
   */
  public double getGyroRaw() {
    return -ahrs.getAngle();
  }

  /**
	 * Zero the gyro position in software to the current angle.
	 */
	public void zeroGyroRotation() {
    yawZero = getGyroRaw(); // set yawZero to gyro angle
  }
  
  /**
	 * Zero the gyro position in software against a specified angle.
	 * @param currentHeading current robot angle compared to the zero angle
	 */
	public void zeroGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
  }

  /**
	 * @return double, gyro angle from 180 to -180, in degrees (postitive is left, negative is right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]
		angle = MathBCR.normalizeAngle(angle);
		return angle;
  }

  /**
   * @return gyro angular velocity (with some averaging to reduce noise), in degrees per second.
   * Positive is turning left, negative is turning right.
   */
  public double getAngularVelocity () {
    return angularVelocity;
  }

  /**
   * @return angular velocity from motor velocity readings (NOT from gyro)
   * Positive is turning left, negative is turning right.
   */
  // public double getAngularVelocityFromWheels () {
    //TODO In the 2022 code, this was more accurate than the angular velocity from
    // the gyro.  This was used in the DriveTurnGyro code.  However, angular velocity
    // was easy to calculate from a west coast driveTrain.  How do we calculate this
    // from a swerve drive train?  Do we need this method?
  //   return ((getRightEncoderVelocity() - getLeftEncoderVelocity()) / 2) * wheelInchesToGyroDegrees;
  // }


  // ************ Swerve drive methods

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    swerveFrontLeft.setMotorModeCoast(setCoast);
    swerveFrontRight.setMotorModeCoast(setCoast);
    swerveBackLeft.setMotorModeCoast(setCoast);
    swerveBackRight.setMotorModeCoast(setCoast);
  }

  /**
   * Stops all of the drive and turning motors
   */
  public void stopMotors() {
    swerveFrontLeft.stopMotors();
    swerveFrontRight.stopMotors();
    swerveBackLeft.stopMotors();
    swerveBackRight.stopMotors();
  }

  /**
   * Sets the swerve ModuleStates.
   * Note from Don -- I believe this method needs to be called repeatedly to function.
   *
   * @param desiredStates The desired SwerveModule states.          
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeedMetersPerSecond);
    swerveFrontLeft.setDesiredState(desiredStates[0]);
    swerveFrontRight.setDesiredState(desiredStates[1]);
    swerveBackLeft.setDesiredState(desiredStates[2]);
    swerveBackRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Method to drive the robot using desired robot velocity and orientation, such as from joystick info.
   * Note from Don -- I believe this method needs to be called repeatedly to function.
   *
   * @param xSpeed Speed of the robot in the x direction, in meters per second (+ = forward)
   * @param ySpeed Speed of the robot in the y direction, in meters per second (- = sideways)
   * @param rot Angular rate of the robot, in radians per second (+ = turn to the left)
   * @param fieldRelative True = the provided x and y speeds are relative to the field.
   * False = the provided x and y speeds are relative to the current facing of the robot. 
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, new Translation2d(), fieldRelative);
  }

  /**
   * Method to drive the robot using desired robot velocity and orientation, such as from joystick info.
   * Note from Don -- I believe this method needs to be called repeatedly to function.
   *
   * @param xSpeed Speed of the robot in the x direction, in meters per second (+ = forward)
   * @param ySpeed Speed of the robot in the y direction, in meters per second (- = sideways)
   * @param rot Angular rate of the robot, in radians per second (+ = turn to the left)
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and XSpeed, YSpeed = 0, then the robot will rotate around that corner.
   *     This feature may not work if fieldRelative = True (need to test).
   * @param fieldRelative True = the provided x and y speeds are relative to the field.
   * False = the provided x and y speeds are relative to the current facing of the robot. 
   */
   public void drive(double xSpeed, double ySpeed, double rot, Translation2d centerOfRotationMeters, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            centerOfRotationMeters);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, kMaxSpeedMetersPerSecond);
    swerveFrontLeft.setDesiredState(swerveModuleStates[0]);
    swerveFrontRight.setDesiredState(swerveModuleStates[1]);
    swerveBackLeft.setDesiredState(swerveModuleStates[2]);
    swerveBackRight.setDesiredState(swerveModuleStates[3]);
  }

  // ************ Odometry methods

  /**
   * Returns the currently-estimated position of the robot on the field
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.  I.e. defines the robot's
   * position and orientation on the field.
   *
   * @param pose The pose to which to set the odometry.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = middle of robot wherever the robot starts auto mode, +=away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = middle of robot wherever the robot starts auto mode, +=left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition( pose,
        Rotation2d.fromDegrees(getGyroRotation()) );
  }

  // ************ Information methods


  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Checks if the CAN bus and gyro are working.  Sometimes, when the robot boots up, either the CAN bus or
   * the gyro don't initialize properly.  ROBOT CODE WILL NOT BE ABLE TO CONTROL MOTORS when this happens, so
   * always check this before starting a match!
   * @return true = something is not working.  false = CAN bus and gyro are both working.
   */
  public boolean canBusError() {
    return ((swerveFrontLeft.getDriveBusVoltage() < 7.0) || (swerveFrontLeft.getDriveTemp() < 5.0) || !isGyroReading());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // save current angle and time for calculating angVel
    currAng = getGyroRaw();
    currTime = System.currentTimeMillis();
 
    // calculate angVel in degrees per second
    angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );

    // Update robot odometry
    double degrees = getGyroRotation();
    odometry.update(Rotation2d.fromDegrees(degrees), new SwerveModuleState[] {
          swerveFrontLeft.getState(), swerveFrontRight.getState(),
          swerveBackLeft.getState(), swerveBackRight.getState()
        });
        
    if(fastLogging || log.getLogRotation() == log.DRIVE_CYCLE) {
      updateDriveLog(false);

      if(!isGyroReading()) {
        RobotPreferences.recordStickyFaults("Gyro", log);
      }

      // Update data on SmartDashboard
      // TODO Add more logging to Shuffleboard!
      // SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      // SmartDashboard.putNumber("Drive Fwd Velocity", getLeftEncoderVelocity());
      // SmartDashboard.putNumber("Drive Sideways Velocity", getRightEncoderVelocity());
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putNumber("Drive Gyro Rotation", degrees);
      SmartDashboard.putNumber("Drive AngVel", angularVelocity);
      SmartDashboard.putNumber("Drive Pitch", ahrs.getRoll());
      
      // position from odometry (helpful for autos)
      Pose2d pose = odometry.getPoseMeters();
      SmartDashboard.putNumber("Drive Odometry X", pose.getTranslation().getX());
      SmartDashboard.putNumber("Drive Odometry Y", pose.getTranslation().getY());
      SmartDashboard.putNumber("Drive Odometry Theta", pose.getRotation().getDegrees());

      //Values for bugfixing
      SmartDashboard.putNumber("Drive Bus Volt", swerveFrontLeft.getDriveBusVoltage());
      SmartDashboard.putNumber("Drive Motor Temp", swerveFrontLeft.getDriveTemp());
    }

    // save current angVel values as previous values for next calculation
    prevAng = currAng;
    prevTime = currTime; 
  }

  /**
   * Writes information about the drive train to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    // TODO Add more logging to Shuffleboard!

    Pose2d pose = odometry.getPoseMeters();
    log.writeLog(logWhenDisabled, "Drive", "Update Variables", 
      // "L1 Volts", leftMotor1.getMotorOutputVoltage(), "L2 Volts", leftMotor2.getMotorOutputVoltage(),
      // "L1 Amps", leftMotor1.getSupplyCurrent(), "L2 Amps", leftMotor2.getSupplyCurrent(),
      // "L1 Temp",leftMotor1.getTemperature(), "L2 Temp",leftMotor2.getTemperature(),
      // "R1 Volts", rightMotor1.getMotorOutputVoltage(), "R2 Volts", rightMotor2.getMotorOutputVoltage(),
      // "R1 Amps", rightMotor1.getSupplyCurrent(), "R2 Amps", rightMotor2.getSupplyCurrent(), 
      // "R1 Temp",rightMotor1.getTemperature(), "R2 Temp",rightMotor2.getTemperature(),
      // "Left Inches", getLeftEncoderInches(), "L Vel", getLeftEncoderVelocity(),
      // "Right Inches", getRightEncoderInches(), "R Vel", getRightEncoderVelocity(),
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), 
      "Gyro Velocity", angularVelocity, "Pitch", ahrs.getRoll(),
      "Odometry X", pose.getTranslation().getX(), "Odometry Y", pose.getTranslation().getY(), 
      "Odometry Theta", pose.getRotation().getDegrees()
      );
  }
}
