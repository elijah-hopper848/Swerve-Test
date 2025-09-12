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
  private static final SparkMaxConfig s_driveConfig = new SparkMaxConfig();

  private final Translation2d m_offset;
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;
  private final CANcoder m_encoder;
  private final PIDController m_turnPID;
  
  public SwerveModule(int driveId, int turnId, int encoderId, Translation2d offset) {
    s_driveConfig.idleMode(IdleMode.kBrake);
    s_driveConfig.smartCurrentLimit(159);
    s_driveConfig.voltageCompensation(12);

    m_driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnId, MotorType.kBrushless);
    m_encoder = new CANcoder(encoderId);

    m_turnPID = new PIDController(1, 1, 1);
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);

    m_driveMotor.configure(s_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(s_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_offset = offset;
  }

  private double getPerpendicularOffsetAngle() {
    return Math.atan2(m_offset.getY(), m_offset.getX()) - (Math.PI / 2);
  }

  public void updateMotors(double leftJoystickX, double leftJoystickY, double rightJoystickX) {
    // calculate the perpendicular angle (angle of the turning vector) of the module's offset
    double turnVectorAngle = getPerpendicularOffsetAngle();
    
    // add scaled turn vector to the raw left joystick input
    double desiredSpeedX = leftJoystickX + Math.cos(turnVectorAngle) * rightJoystickX;
    double desiredSpeedY = leftJoystickY + Math.sin(turnVectorAngle) * rightJoystickX;

    // calculate desired angle and speed for motors
    double desiredAngle = Math.atan2(desiredSpeedY, desiredSpeedX);
    double desiredSpeed = Math.hypot(desiredSpeedX, desiredSpeedY);
    
    // update the motors
    m_turnMotor.setVoltage(m_turnPID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble(), desiredAngle));
    m_driveMotor.set(desiredSpeed);
  }
}
