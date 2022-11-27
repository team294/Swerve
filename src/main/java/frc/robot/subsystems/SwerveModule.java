// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
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
   * @param driveMotorAddress The CANbus address of the drive motor.
   * @param turningMotorAddress The CANbus address of the turning motor.
   * @param turningEncoderAddress The CANbus address of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   * @param turningOffsetDegrees Offset degrees in the turning motor to point to the 
   * front of the robot.  Value is the desired encoder zero point, in absolute magnet position reading.
   */
  public SwerveModule(int driveMotorAddress, int turningMotorAddress, int turningEncoderAddress,
      boolean driveEncoderReversed, boolean turningEncoderReversed,
      double turningOffsetDegrees) {

    driveMotor = new WPI_TalonFX(driveMotorAddress);
    turningMotor = new WPI_TalonFX(turningMotorAddress);

    turningEncoder = new WPI_CANCoder(turningEncoderAddress);

    driveMotor.configFactoryDefault();
    turningMotor.configFactoryDefault();
    turningEncoder.configFactoryDefault();

    driveMotor.setInverted(false);
    turningMotor.setInverted(false);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);

    driveMotor.configNeutralDeadband(0.0);
    turningMotor.configNeutralDeadband(0.0);

    driveMotor.configVoltageCompSaturation(ModuleConstants.compensationVoltage);
    turningMotor.configVoltageCompSaturation(ModuleConstants.compensationVoltage);

    driveMotor.enableVoltageCompensation(true);
    turningMotor.enableVoltageCompensation(true);

    driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    // Set whether the encoder should be reversed or not
    driveMotor.setSensorPhase(driveEncoderReversed);
    turningEncoder.configSensorDirection(turningEncoderReversed);

    turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    calibrateTurningEncoderDegrees(turningOffsetDegrees);

    //TODO Stopped updates here in the constructor.  What else to do here?

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
   * Sets the desired state for the module.
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

  public double getTurningStatorCurrent() {
    return turningMotor.getStatorCurrent();
  }

  public double getTurningTemp() {
    return turningMotor.getTemperature();
  }


}
