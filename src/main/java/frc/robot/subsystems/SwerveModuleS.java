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

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleS {
  private static final SparkMaxConfig driveConfig = new SparkMaxConfig();

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;
  private final Translation2d position;
  
  public SwerveModuleS(int driveId, int turnId, Translation2d offset) {
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.smartCurrentLimit(159);
    driveConfig.voltageCompensation(12);

    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    turnMotor = new SparkMax(turnId, MotorType.kBrushless);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    position = offset;
  }

  public void set(double leftJoystickX, double leftJoystickY, double rightJoystickX) {
    // physics stuff HELP PLS GOD
  }
}
