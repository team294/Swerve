// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ModuleConstants;
import static frc.robot.utilities.StringUtil.*;

public class SwerveModule {
      
  private final String swName;    // Name for this swerve module

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  private final WPI_CANCoder turningEncoder;

  private final PIDController drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  
  private double driveEncoderZero = 0;

  /**
   * Constructs a SwerveModule.
   *
   * @param swName The name of this swerve module, for use in Shuffleboard and logging
   * @param driveMotorAddress The CANbus address of the drive motor.
   * @param turningMotorAddress The CANbus address of the turning motor.
   * @param turningEncoderAddress The CANbus address of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   * @param turningOffsetDegrees Offset degrees in the turning motor to point to the 
   * front of the robot.  Value is the desired encoder zero point, in absolute magnet position reading.
   */
  public SwerveModule(String swName, int driveMotorAddress, int turningMotorAddress, int turningEncoderAddress,
      boolean driveEncoderReversed, boolean turningEncoderReversed,
      double turningOffsetDegrees) {

    // Save the module name
    this.swName = swName;

    // Create motor and encoder objects
    driveMotor = new WPI_TalonFX(driveMotorAddress);
    turningMotor = new WPI_TalonFX(turningMotorAddress);
    turningEncoder = new WPI_CANCoder(turningEncoderAddress);

    // configure drive motor
    driveMotor.configFactoryDefault();
    driveMotor.setInverted(false);
    driveMotor.configNeutralDeadband(0.0);
    driveMotor.configVoltageCompSaturation(ModuleConstants.compensationVoltage);
    driveMotor.enableVoltageCompensation(true);

    // configure turning motor
    turningMotor.configFactoryDefault();
    turningMotor.setInverted(false);
    turningMotor.configNeutralDeadband(0.0);
    turningMotor.configVoltageCompSaturation(ModuleConstants.compensationVoltage);
    turningMotor.enableVoltageCompensation(true);

    // other configs for drive and turning motors
    setMotorModeCoast(true);        // true on boot up, so robot is easy to push.  Change to false in autoinit or teleopinit

    // configure drive encoder
    driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    driveMotor.setSensorPhase(driveEncoderReversed);

    // configure turning encoder
    turningEncoder.configFactoryDefault();
    turningEncoder.configSensorDirection(turningEncoderReversed);
    turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    calibrateTurningEncoderDegrees(turningOffsetDegrees);

    //TODO Stopped updates here in the constructor.  What else to do here?

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ********** Swerve module configuration methods

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setMotorModeCoast(boolean setCoast) {
    if (setCoast) {
      driveMotor.setNeutralMode(NeutralMode.Coast);
      turningMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralMode.Brake);
      turningMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public void setDriveMotorPercentOutput(double percentOutput){
    driveMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setTurnMotorPercentOutput(double percentOutput){
    turningMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  // ********** Main swerve module control methods

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveEncoderVelocity(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  // public SwerveModulePosition getPosition() {
  //   return new SwerveModulePosition(
  //       getDriveEncoderMeters(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  // }

  /**
   * Turns off the drive and turning motors.
   */
  public void stopMotors() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  /**
   * Sets the desired state for the module.
   * Note from Don -- I believe this method needs to be called repeatedly to function.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningEncoderDegrees()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(getDriveEncoderVelocity(), state.speedMetersPerSecond);

    //TODO is this right?  Do we need to call reset()?  -Don
    //TODO Is it better to use the PID controller on the Falcon to just set the wheel to an orientation?
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        turningPIDController.calculate(getTurningEncoderDegrees() * Math.PI/180.0, state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    turningMotor.set(turnOutput);
  }

  // ********** Encoder methods

  /**
   * @return drive encoder position, in ticks
   */
  public double getDriveEncoderRaw() {
    return driveMotor.getSelectedSensorPosition(0);
  }

  /**
	 * Zero the drive encoder position in software.
	 */
  public void zeroDriveEncoder() {
    driveEncoderZero = getDriveEncoderRaw();
  }

  /**
   * @return drive encoder position, in meters (+ = forward)
   */
  public double getDriveEncoderMeters() {
    //TODO Calibrate the constant for this calculation
    return (getDriveEncoderRaw() - driveEncoderZero) * ModuleConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * @return drive encoder velocity, in ticks per 100ms (+ = forward)
   */
  public double getDriveEncoderVelocityRaw() {
    return driveMotor.getSelectedSensorVelocity(0);
  }

  /**
   * @return drive encoder velocity, in meters per second (+ = forward)
   */
  public double getDriveEncoderVelocity() {
    //TODO Verify that the output is correct
    return getDriveEncoderVelocityRaw() * ModuleConstants.kDriveEncoderMetersPerTick * 10.0;
  }


  /**
   * Calibrates the turning encoder, so that 0 should be with the wheel pointing toward the front of robot.
   * @param turningOffsetDegrees Desired encoder zero point, in absolute magnet position reading
   */
  public void calibrateTurningEncoderDegrees(double turningOffsetDegrees) {
    turningEncoder.configMagnetOffset(turningOffsetDegrees);
  }

  /**
   * @return turning encoder position, in degrees [-180,+180).
   * When calibrated, 0 should be with the wheel pointing toward the front of robot.
   * + = counterclockwise, - = clockwise
   */
  public double getTurningEncoderDegrees() {
    //TODO Verify that the sign is correct, per the JavaDoc above.  If not, fix the code below (JavaDoc is the correct intent).
    return turningEncoder.getPosition();
  }

  /**
   * @return turning encoder position, in degrees per second (-)
   */
  public double getTurningEncoderVelocityDPS() {
    //TODO Verify that the output is correct
    return turningEncoder.getVelocity();
  }

  // ********** Information methods

  public double getDriveBusVoltage() {
    return driveMotor.getBusVoltage();
  }

  public double getDriveOutputVoltage() {
    return driveMotor.getMotorOutputVoltage();
  }

  public double getDriveOutputPercent() {
    return driveMotor.getMotorOutputPercent();
  }

  public double getDriveStatorCurrent() {
    return driveMotor.getStatorCurrent();
  }

  public double getDriveTemp() {
    return driveMotor.getTemperature();
  }

  public double getTurningOutputVoltage() {
    return turningMotor.getMotorOutputVoltage();
  }

  public double getTurningOutputPercent() {
    return turningMotor.getMotorOutputPercent();
  }

  public double getTurningStatorCurrent() {
    return turningMotor.getStatorCurrent();
  }

  public double getTurningTemp() {
    return turningMotor.getTemperature();
  }

  /**
   * Updates relevant variables on Shuffleboard
   */
  public void updateShuffleboard() {
    SmartDashboard.putNumber(buildString("Swerve angle ", swName), getTurningEncoderDegrees());
    SmartDashboard.putNumber(buildString("Swerve drive temp ", swName), getDriveTemp());
  }

  /**
   * Returns information about the swerve module to include in the filelog
   * Format of the return string is comma-delimited name-value pairs, 
   * *without* the final comma.  Ex.  "name1,value1,name2,value2"
   */
  public String getLogString() {
    return buildString(
      swName, " angle deg,", getTurningEncoderDegrees(), ",",
      swName, " angle DPS,", getTurningEncoderVelocityDPS(), ",",
      swName, " turn output,", getTurningOutputPercent(), ",",
      swName, " drive meters,", getDriveEncoderMeters(), ",",
      swName, " drive mps,", getDriveEncoderVelocity(), ",",
      swName, " drive output,", getDriveOutputPercent(), ",",
      swName, " drive temp,", getDriveTemp(), ",",
      swName, " turn temp,", getTurningTemp()
    );
  }
}
