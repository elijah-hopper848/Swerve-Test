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
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModule {
  private static final SparkMaxConfig driveConfig = new SparkMaxConfig();

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;
  private final Translation2d position;
  private final CANcoder encoder;
  private final PIDController turnPID;
  
  public SwerveModule(int driveId, int turnId, int encoderid, Translation2d offset) {
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.smartCurrentLimit(159);
    driveConfig.voltageCompensation(12);

    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    turnMotor = new SparkMax(turnId, MotorType.kBrushless);
    encoder = new CANcoder(encoderid);

    turnPID = new PIDController(1, 1, 1);
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    position = offset;
  }

  public void updateMotors(double leftJoystickX, double leftJoystickY, double rightJoystickX) {
    double rotationVectorAngle = Math.atan2(position.getY(), position.getX()) - (Math.PI / 2);
    
    double desiredSpeedX = leftJoystickX + Math.cos(rotationVectorAngle) * rightJoystickX;
    double desiredSpeedY = leftJoystickY + Math.sin(rotationVectorAngle) * rightJoystickX;

    double desiredAngle = Math.atan2(desiredSpeedY, desiredSpeedX);
    double desiredSpeed = Math.hypot(desiredSpeedX, desiredSpeedY);

    turnMotor.setVoltage(turnPID.calculate(encoder.getAbsolutePosition().getValueAsDouble(), desiredAngle));
    driveMotor.set(desiredSpeed);
  }
}
