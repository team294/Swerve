/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.*;

/**
 * Command to control the drive train with joysticks using arcade drive.
 */
public class DriveWithJoystick extends CommandBase {
  private final DriveTrain driveTrain;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final FileLog log;
  
  private double fwdPercent, leftPercent, turnPercent;
  // private double lastFwdPercent, lastTime, curTime;

  // private final double maxFwdRateChange = 2.0;
  // private final double maxRevRateChange = -1.4;

  /**
   * @param driveTrain drive train subsystem to use
   * @param leftJoystick left joystick.  X and Y axis control robot movement, relative to front of robot
   * @param rightJoystick right joystick.  X-axis controls robot rotation.
   * @param log filelog to use
   */
  public DriveWithJoystick(DriveTrain driveTrain, Joystick leftJoystick, Joystick rightJoystick, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    // lastFwdPercent = 0;
    // lastTime = System.currentTimeMillis() / 1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // curTime = System.currentTimeMillis() / 1000.0;
    fwdPercent = -leftJoystick.getY();
    leftPercent = -leftJoystick.getX();
    turnPercent = rightJoystick.getX() * 0.5;

    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      log.writeLog(false, "DriveWithJoystickArcade", "Joystick", "Fwd", fwdPercent, "Left", leftPercent, "Turn", turnPercent);
    }

    // double fwdRateChange = (fwdPercent - lastFwdPercent) / (curTime - lastTime);
    // if (fwdRateChange > maxFwdRateChange) {
    //   fwdPercent = lastFwdPercent + (curTime - lastTime)*maxFwdRateChange;
    // } else if (fwdRateChange < maxRevRateChange) {
    //   fwdPercent = lastFwdPercent +(curTime - lastTime)*maxRevRateChange;

    // }
    
    driveTrain.drive(fwdPercent, leftPercent, turnPercent, false);

    // lastFwdPercent = fwdPercent;
    // lastTime = curTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}