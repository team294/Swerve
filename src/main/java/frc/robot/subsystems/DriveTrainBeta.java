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
// import static frc.robot.Constants.RobotConstants.*; 
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.Ports.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrainBeta extends SubsystemBase {
  
  //motor modules
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;


  private final WPI_TalonFX leftMotor1;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX rightMotor1;
  private final WPI_TalonFX rightMotor2;

  //variables for steer motors
  private final WPI_TalonFX leftSteerMotor1;
  private final WPI_TalonFX leftSteerMotor2;
  private final WPI_TalonFX rightSteerMotor1;
  private final WPI_TalonFX rightSteerMotor2;

  private final DifferentialDriveOdometry odometry;

  private final AHRS ahrs;
  private double yawZero = 0;

  private FileLog log;
  
  
  // variables to help calculate angular velocity for turnGyro
  private double prevAng; // last recorded gyro angle
  private double currAng; // current recorded gyro angle
  private double prevTime; // last time gyro angle was recorded
  private double currTime; // current time gyro angle is being recorded
  private double angularVelocity;  // Robot angular velocity in degrees per second
  private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc


/** Creates a new Drivetrain. */
  public DriveTrainBeta(FileLog log) {
    this.log = log; // save reference to the fileLog
    

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
            CANDriveFrontLeftMotor,
            // This is the ID of the steer motor
            CANDriveTurnFrontLeftMotor,
            // This is the ID of the steer encoder
            CANTurnEncoderFrontLeft,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            CANDriveFrontRightMotor,
            CANDriveTurnFrontRightMotor,
            CANTurnEncoderFrontRight,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            CANDriveBackLeftMotor,
            CANDriveTurnBackLeftMotor,
            CANTurnEncoderBackLeft,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD,
            CANDriveBackRightMotor,
            CANDriveTurnBackRightMotor,
            CANTurnEncoderBackRight,
            BACK_RIGHT_MODULE_STEER_OFFSET
    ); 
    
    leftMotor1 = m_frontLeftModule.getDriveMotor();
    leftMotor2 = m_backLeftModule.getDriveMotor();
    rightMotor1 = m_frontRightModule.getDriveMotor();
    rightMotor2 = m_frontRightModule.getDriveMotor();

    leftMotor1.configFactoryDefault();
    leftMotor2.configFactoryDefault();
    rightMotor1.configFactoryDefault();
    rightMotor2.configFactoryDefault();

    leftMotor2.set(ControlMode.Follower, canLeftDriveMotor1);
    rightMotor2.set(ControlMode.Follower, canRightDriveMotor1);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    //tieing left&right steer motors, need to toggle inversion 
    leftSteerMotor1 = m_frontLeftModule.getSteerMotor();
    leftSteerMotor2 = m_backLeftModule.getSteerMotor();
    rightSteerMotor1 = m_frontRightModule.getSteerMotor();
    rightSteerMotor2 = m_frontRightModule.getSteerMotor();
    
    leftSteerMotor1.configFactoryDefault();
    leftSteerMotor2.configFactoryDefault();
    rightSteerMotor1.configFactoryDefault();
    rightSteerMotor2.configFactoryDefault();

    rightSteerMotor1.set(ControlMode.Follower, canLeftSteerMotor1);
    rightSteerMotor2.set(ControlMode.Follower, canLeftSteerMotor2);

    rightSteerMotor1.follow(leftSteerMotor1);
    rightSteerMotor2.follow(leftSteerMotor2);

    // Drive train is reversed on competition robot
    if (prototypeBot) {
      leftMotor1.setInverted(true);
      leftMotor2.setInverted(true);
      rightMotor1.setInverted(true);
      rightMotor2.setInverted(true);
    } else { 
      leftMotor1.setInverted(false);
      leftMotor2.setInverted(false);
      rightMotor1.setInverted(true);
      rightMotor2.setInverted(true);
      // rightMotor1.setInverted(false);
      // rightMotor2.setInverted(false);
    }

    setDriveModeCoast(false);

    leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    leftMotor1.setSensorPhase(false);
    rightMotor1.setSensorPhase(false);

    leftMotor1.configNeutralDeadband(0.0);
    leftMotor2.configNeutralDeadband(0.0);
    rightMotor1.configNeutralDeadband(0.0);
    rightMotor2.configNeutralDeadband(0.0);

    leftMotor1.configVoltageCompSaturation(compensationVoltage);
    leftMotor2.configVoltageCompSaturation(compensationVoltage);
    rightMotor1.configVoltageCompSaturation(compensationVoltage);
    rightMotor2.configVoltageCompSaturation(compensationVoltage);

    // Change number of samples in the rolling average for voltage compensation (default = 12?)
    // leftMotor1.configVoltageMeasurementFilter(4);
    // leftMotor2.configVoltageMeasurementFilter(4);
    // rightMotor1.configVoltageMeasurementFilter(4);
    // rightMotor2.configVoltageMeasurementFilter(4);

    setVoltageCompensation(true);
    setOpenLoopRampLimit(true);
    
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


  /**
   * @param percent percent output (+1 = forward, -1 = reverse)
   */
  public void setLeftMotorOutput(double percent) {
    leftMotor1.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @param percent percent output (+1 = forward, -1 = reverse)
   */
  public void setRightMotorOutput(double percent) {
    rightMotor1.set(ControlMode.PercentOutput, percent);
  }

  /**
   * @param turnOn true = turn on voltage compensation, false = turn off voltage compensation
   */
  public void setVoltageCompensation(boolean turnOn) {
    leftMotor1.enableVoltageCompensation(turnOn);
    leftMotor2.enableVoltageCompensation(turnOn);
    rightMotor1.enableVoltageCompensation(turnOn);
    rightMotor2.enableVoltageCompensation(turnOn);
  }
  
  /**
   * @param turnOn true = turn on open loop ramp rate limit, false = turn off open loop ramp rate limit
   */
  public void setOpenLoopRampLimit(boolean turnOn) {
    double ramp = turnOn ? 0.4 : 0.0;
    leftMotor1.configOpenloopRamp(ramp);
    leftMotor2.configOpenloopRamp(ramp);
    rightMotor1.configOpenloopRamp(ramp);
    rightMotor2.configOpenloopRamp(ramp);
  }

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    if (setCoast) {
      leftMotor1.setNeutralMode(NeutralMode.Coast);
      leftMotor2.setNeutralMode(NeutralMode.Coast);
      rightMotor1.setNeutralMode(NeutralMode.Coast);
      rightMotor2.setNeutralMode(NeutralMode.Coast);
    } else {
      leftMotor1.setNeutralMode(NeutralMode.Brake);
      leftMotor2.setNeutralMode(NeutralMode.Brake);
      rightMotor1.setNeutralMode(NeutralMode.Brake);
      rightMotor2.setNeutralMode(NeutralMode.Brake);
    }
  }


  /**
   * @return average velocity, in meters per second
   */
  public double getAverageEncoderVelocity(){
    return (getRightEncoderVelocity() + getLeftEncoderVelocity()) / 2;
  }


  /**
   * Gets the raw gyro angle (can be greater than 360).
   * Angle is negated from the gyro, so that + = left and - = right
   * <p> From the WPILib doc for odometry:  As your robot turns to the left, 
   * your gyroscope angle should increase. By default, WPILib gyros exhibit the 
   * opposite behavior, so you should negate the gyro angle.
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
   * Verifies if Gyro is still reading
   * @return true = gryo is connected to Rio
   */
  public boolean isGyroReading() {
    return ahrs.isConnected();
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
   */
  public double getAngularVelocityFromWheels () {
    return ((getRightEncoderVelocity() - getLeftEncoderVelocity()) / 2) * wheelInchesToGyroDegrees;
  }

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

  /**
   * Stops motors by calling tankDrive(0, 0).
   */
  public void stop() {
    tankDrive(0.0, 0.0);
  }

  /**
   * @return average between left and right encoders, in inches
   */
  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
  }

  /**
   * Set up PID parameters for the drive train Talons
   * @param kP Proportional term
   * @param kI Integral term 
   * @param kD Differential term
   * @param kF Feed forward term (multiplied by setpoint)
   */
  public void setTalonPIDConstants(double kP, double kI, double kD, double kF) {
    leftMotor1.config_kP(0, kP);
    leftMotor1.config_kI(0, kI);
    leftMotor1.config_kD(0, kD);
    leftMotor1.config_kF(0, kF);

    rightMotor1.config_kP(0, kP);
    rightMotor1.config_kI(0, kI);
    rightMotor1.config_kD(0, kD);
    rightMotor1.config_kF(0, kF);

    SmartDashboard.putNumber("Drive kP Linear", kP);
    SmartDashboard.putNumber("Drive kI Linear", kI);
    SmartDashboard.putNumber("Drive kD Linear", kD);

    leftMotor1.selectProfileSlot(0, 0);
    rightMotor1.selectProfileSlot(0, 0);
  }

  /**
   * Resets the Talon PIDs.  Use this when re-starting the PIDs.
   */
  public void resetTalonPIDs() {
    leftMotor1.setIntegralAccumulator(0);
    rightMotor1.setIntegralAccumulator(0);
  }

  /**
   * Sets the left Talon to velocity closed-loop control mode with target velocity and feed-forward constant.
   * @param targetVel Target velocity, in inches per second
   * @param aFF Feed foward term to add to the contorl loop (-1 to +1)
   */
  public void setLeftTalonPIDVelocity(double targetVel, double aFF) {
    leftMotor1.set(ControlMode.Velocity, 
      targetVel * ticksPerInch / 10.0, DemandType.ArbitraryFeedForward, aFF);
  }

  /**
   * Sets the right Talon to velocity closed-loop control mode with target velocity and feed-forward constant.
   * @param targetVel Target velocity, in inches per second
   * @param aFF Feed foward term to add to the contorl loop (-1 to +1)
   * @param reverseRight True = reverse velocity and FF term for right Talon
   */
  public void setRightTalonPIDVelocity(double targetVel, double aFF, boolean reverseRight) {
    int direction = (reverseRight) ? 1 : -1;
    rightMotor1.set(ControlMode.Velocity, 
      targetVel*direction  * ticksPerInch / 10.0, DemandType.ArbitraryFeedForward, aFF*direction);
  }


  public double getTalonLeftClosedLoopError() {
    return leftMotor1.getClosedLoopError();
  }

  public double getTalonLeftClosedLoopTarget() {
    return leftMotor1.getClosedLoopTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update robot odometry
    double degrees = getGyroRotation();
    double leftMeters = Units.inchesToMeters(getLeftEncoderInches());
    double rightMeters = Units.inchesToMeters(getRightEncoderInches());
    odometry.update(Rotation2d.fromDegrees(degrees), leftMeters, rightMeters);

    // save current angle and time for calculating angVel
    currAng = getGyroRaw();
    currTime = System.currentTimeMillis();
 
    // calculate angVel in degrees per second
    angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );
     
    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      updateDriveLog(false);

      if(!isGyroReading()) {
        RobotPreferences.recordStickyFaults("Gyro", log);
      }

      // read PID coefficients from SmartDashboard
      kVLinear = SmartDashboard.getNumber("Drive kV Linear", kVLinear);
      kALinear = SmartDashboard.getNumber("Drive kA Linear", kALinear);
      kSLinear = SmartDashboard.getNumber("Drive kS Linear", kSLinear);
      kPLinear = SmartDashboard.getNumber("Drive kP Linear", kPLinear);
      kILinear = SmartDashboard.getNumber("Drive kI Linear", kILinear);
      kDLinear = SmartDashboard.getNumber("Drive kD Linear", kDLinear);
      kAngLinear = SmartDashboard.getNumber("Drive kAng Linear", kAngLinear);

      kVAngular = SmartDashboard.getNumber("Drive kV Angular", kVAngular);
      kAAngular = SmartDashboard.getNumber("Drive kA Angular", kAAngular);
      kSAngular = SmartDashboard.getNumber("Drive kS Angular", kSAngular);
      kPAngular = SmartDashboard.getNumber("Drive kP Angular", kPAngular);
      kIAngular = SmartDashboard.getNumber("Drive kI Angular", kIAngular);
      kDAngular = SmartDashboard.getNumber("Drive kD Angular", kDAngular);
      tLagAngular = SmartDashboard.getNumber("Drive tLag Angular", tLagAngular);
       
      // Update data on SmartDashboard
      SmartDashboard.putNumber("Drive Right Raw", getRightEncoderRaw());
      SmartDashboard.putNumber("Drive Left Raw", getLeftEncoderRaw());
      SmartDashboard.putNumber("Drive Right Enc", getRightEncoderInches());
      SmartDashboard.putNumber("Drive Left Enc", getLeftEncoderInches());
      SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      SmartDashboard.putNumber("Drive Left Velocity", getLeftEncoderVelocity());
      SmartDashboard.putNumber("Drive Right Velocity", getRightEncoderVelocity());
      SmartDashboard.putNumber("Drive Gyro Rotation", degrees);
      SmartDashboard.putNumber("Drive AngVel", angularVelocity);
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());

      // position from odometry (helpful for autos)
      var translation = odometry.getPoseMeters().getTranslation();
      SmartDashboard.putNumber("Drive Odometry X",translation.getX());
      SmartDashboard.putNumber("Drive Odometry Y",translation.getY());

      //Values for bugfixing
      SmartDashboard.putNumber("Drive Motor Temp", leftMotor1.getTemperature());
      SmartDashboard.putNumber("Drive Bus Volt", leftMotor1.getBusVoltage());
    }

    // save current angVel values as previous values for next calculation
    prevAng = currAng;
    prevTime = currTime; 
  }

  /**
   * Get current robot location and facing on the field
   * @return current robot pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the robot pose on the field to the given location and rotation
   * <p>Note:  This method resets the encoders to 0 and sets the gyro
   * to the current robot rotation.
   * <p>Robot X: 0 = middle of robot, wherever the robot starts auto mode (+=away from our drivestation)
   * <p>Robot Y: 0 = middle of robot, wherever the robot starts auto mode (+=left when looking from our drivestation)
   * <p>Robot angle: 0 = facing away from our drivestation
   * @param robotPoseInMeters Current robot pose, in meters
   */
  public void resetPose(Pose2d robotPoseInMeters) {
    zeroLeftEncoder();
    zeroRightEncoder();
    zeroGyroRotation(robotPoseInMeters.getRotation().getDegrees());
    odometry.resetPosition(robotPoseInMeters, Rotation2d.fromDegrees(getGyroRotation()));
  }

  /**
   * @return wheel speeds, in meters per second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(getLeftEncoderVelocity()), Units.inchesToMeters(getRightEncoderVelocity()));
  }

  /**
   * Writes information about the drive train to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    var translation = odometry.getPoseMeters().getTranslation();
    log.writeLog(logWhenDisabled, "Drive", "updates", 
      "L1 Volts", leftMotor1.getMotorOutputVoltage(), "L2 Volts", leftMotor2.getMotorOutputVoltage(),
      "L1 Amps", leftMotor1.getSupplyCurrent(), "L2 Amps", leftMotor2.getSupplyCurrent(),
      "L1 Temp",leftMotor1.getTemperature(), "L2 Temp",leftMotor2.getTemperature(),
      "R1 Volts", rightMotor1.getMotorOutputVoltage(), "R2 Volts", rightMotor2.getMotorOutputVoltage(),
      "R1 Amps", rightMotor1.getSupplyCurrent(), "R2 Amps", rightMotor2.getSupplyCurrent(), 
      "R1 Temp",rightMotor1.getTemperature(), "R2 Temp",rightMotor2.getTemperature(),
      "Left Inches", getLeftEncoderInches(), "L Vel", getLeftEncoderVelocity(),
      "Right Inches", getRightEncoderInches(), "R Vel", getRightEncoderVelocity(),
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), 
      "Gyro Velocity", angularVelocity, 
      "Odometry X", translation.getX(), "Odometry Y", translation.getY()
      );
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}