// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveSetState extends CommandBase {

  private DriveTrain driveTrain;
  private FileLog log;
  private double driveSpeed, turnSpeed;

  /**
   * Sets the state of all 4 swerve motors
   * @param driveTrain
   * @param driveSpeed speed for drive motors, in meters/sec
   * @param turnSpeed speed for turn motors, in deg/sec
   * @param log
   */
  public DriveSetState(DriveTrain driveTrain, double driveSpeed, double turnSpeed, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.driveSpeed = driveSpeed;
    this.turnSpeed = turnSpeed;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO driveSpeed does not work
    driveTrain.setTurningMotorsVelocity(turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
