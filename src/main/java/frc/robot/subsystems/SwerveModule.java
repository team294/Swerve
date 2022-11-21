// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final WPI_CANCoder turningEncoder;
  //need encoder from talon for driveEncoder
  private final WPI_TalonFX driveEcon
  private final Falcon driveMotor;
  private final Falcon turningMotor;
  
  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorAddress, int turningMotorAddress, int turningEncoderAddress, boolean turningEncoderReverse) {

  
    WPI_CANCoder turningEncoder = new WPI_CANCoder(deviceNumber)
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
