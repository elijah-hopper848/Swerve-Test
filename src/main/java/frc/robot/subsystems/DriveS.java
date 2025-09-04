// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveS extends SubsystemBase {
  private static final SparkMaxConfig driveConfig = new SparkMaxConfig();

  private static final SparkMax driveFL = new SparkMax(Constants.MotorIds.driveFLMotorId, MotorType.kBrushless);
  private static final SparkMax driveFR = new SparkMax(Constants.MotorIds.driveFRMotorId, MotorType.kBrushless);
  private static final SparkMax driveBL = new SparkMax(Constants.MotorIds.driveBLMotorId, MotorType.kBrushless);
  private static final SparkMax driveBR = new SparkMax(Constants.MotorIds.driveBRMotorId, MotorType.kBrushless);
  
  public DriveS() {
    driveConfig.idleMode(IdleMode.kBrake);
    
    driveConfig.voltageCompensation(12);

    driveFL.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveFR.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveBL.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveBR.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void drive(double joystickLeftY, double joystickRightY) {
    // homemade way
    driveFL.set(joystickLeftY);
    driveBL.set(joystickLeftY);
    driveFR.set(joystickRightY);
    driveBR.set(joystickRightY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
