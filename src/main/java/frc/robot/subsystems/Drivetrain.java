// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.utilities.*;


public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  
  private FileLog log;



  public DriveTrain(FileLog log) {
    this.log = log;
    //TODO create for swerve modules
    m_frontLeftModule = new SwerveModule (CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANTurnEncoderFrontLeft, false, false, offsetAngleFrontLeftMotor);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
