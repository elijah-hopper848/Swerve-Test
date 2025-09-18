// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModule {
  private final Translation2d m_offset;
  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;
  private final CANcoder m_encoder;
  private final PIDController m_turnPID;
  
  public SwerveModule(int driveId, int turnId, int encoderId, Translation2d offset) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Example configurations
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40; // Amps
    config.Voltage.PeakForwardVoltage = 10.0;
    config.Voltage.PeakReverseVoltage = -10.0;

    
    m_driveMotor = new TalonFX(driveId);
    m_turnMotor = new TalonFX(turnId);
    m_encoder = new CANcoder(encoderId);
    
    m_turnPID = new PIDController(20, 0, 0);
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    
    m_driveMotor.getConfigurator().apply(config);
    m_turnMotor.getConfigurator().apply(config);

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
    
    // System.out.println(desiredSpeed);

    // update the motors
    m_turnMotor.setVoltage(m_turnPID.calculate(m_encoder.getAbsolutePosition().getValueAsDouble() * Math.PI, desiredAngle));
    m_driveMotor.set(0.3);
  }
}
