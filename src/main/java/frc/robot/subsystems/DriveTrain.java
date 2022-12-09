// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.FileLog;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.utilities.*;


public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final SwerveModule swerveFrontLeft;
  private final SwerveModule swerveFrontRight;
  private final SwerveModule swerveBackLeft;
  private final SwerveModule swerveBackRight;
  
  private FileLog log;

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

    // create for swerve modules
    swerveFrontLeft = new SwerveModule(CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANTurnEncoderFrontLeft, false, false, offsetAngleFrontLeftMotor);
    swerveFrontRight = new SwerveModule(CANDriveFrontRightMotor, CANDriveTurnFrontRightMotor, CANTurnEncoderFrontRight, false, false, offsetAngleFrontRightMotor);
    swerveBackLeft = new SwerveModule(CANDriveBackLeftMotor, CANDriveTurnBackLeftMotor, CANTurnEncoderBackLeft, false, false, offsetAngleBackLeftMotor);
    swerveBackRight = new SwerveModule(CANDriveBackRightMotor, CANDriveTurnBackRightMotor, CANTurnEncoderBackRight, false, false, offsetAngleBackRightMotor);

    //TODO create and initialize odometery
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
	 * @return gyro angle from 180 to -180, in degrees (postitive is left, negative is right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]
		angle = normalizeAngle(angle);
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

  /**
	 * Converts input angle to a number between -179.999 and +180.0.
	 * @return normalized angle
	 */
	public double normalizeAngle(double angle) {
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
  }

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



  // ************ Odometry methods



  // ************ Information methods

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
    // double leftMeters = Units.inchesToMeters(getLeftEncoderInches());
    // double rightMeters = Units.inchesToMeters(getRightEncoderInches());
    // odometry.update(Rotation2d.fromDegrees(degrees), leftMeters, rightMeters);
    
    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      updateDriveLog(false);

      if(!isGyroReading()) {
        RobotPreferences.recordStickyFaults("Gyro", log);
      }

      // Update data on SmartDashboard
      // SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      // SmartDashboard.putNumber("Drive Fwd Velocity", getLeftEncoderVelocity());
      // SmartDashboard.putNumber("Drive Sideways Velocity", getRightEncoderVelocity());
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putNumber("Drive Gyro Rotation", degrees);
      SmartDashboard.putNumber("Drive AngVel", angularVelocity);
      SmartDashboard.putNumber("Drive Pitch", ahrs.getRoll());
      
      // position from odometry (helpful for autos)
      // var translation = odometry.getPoseMeters().getTranslation();
      // SmartDashboard.putNumber("Drive Odometry X",translation.getX());
      // SmartDashboard.putNumber("Drive Odometry Y",translation.getY());

      //Values for bugfixing
      // SmartDashboard.putNumber("Drive Bus Volt", leftMotor1.getBusVoltage());
      // SmartDashboard.putNumber("Drive Motor Temp", leftMotor1.getTemperature());
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
    // var translation = odometry.getPoseMeters().getTranslation();
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
      "Gyro Velocity", angularVelocity, "Pitch", ahrs.getRoll()
      // "Odometry X", translation.getX(), "Odometry Y", translation.getY()
      );
  }
}
