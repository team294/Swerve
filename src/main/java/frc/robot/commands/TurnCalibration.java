// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class TurnCalibration extends CommandBase {

  private DriveTrain driveTrain;
  private FileLog log;
  private double percentOutput, maxPercentOutput, rampTime, rampRate;
  private final Timer timer = new Timer();
  /** Creates a new DriveCalibration. */
  /**
   * 
   * @param driveTrain 
   * @param maxPercentOutput % output (between 0 and 1)
   * @param rampTime ramp up time in seconds
   * @param rampRate Ramp rate in pctOut/second 
   * @param log
   */

  public TurnCalibration(DriveTrain driveTrain, double maxPercentOutput, double rampTime, double rampRate, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.maxPercentOutput = maxPercentOutput;
    this.rampTime = rampTime;
    this.rampRate = rampRate;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();
    percentOutput = MathUtil.clamp(currTime*rampRate, -maxPercentOutput, maxPercentOutput);
    driveTrain.setTurningMotorsOutput(percentOutput);
    driveTrain.enableFastLogging(true);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setDriveModeCoast(true);
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(rampTime);
  }
}
