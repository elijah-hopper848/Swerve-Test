// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveS extends SubsystemBase {
  // define a new SwerveModule for each wheel
  private static final SwerveModule moduleFL = new SwerveModule(Constants.MotorIds.driveFL, Constants.MotorIds.turnFL, Constants.EncoderIds.turnEncoderFL, new Translation2d(-1, 1));
  private static final SwerveModule moduleFR = new SwerveModule(Constants.MotorIds.driveFR, Constants.MotorIds.turnFR, Constants.EncoderIds.turnEncoderFR, new Translation2d(1,1));
  private static final SwerveModule moduleBL = new SwerveModule(Constants.MotorIds.driveBL, Constants.MotorIds.turnBL, Constants.EncoderIds.turnEncoderBL, new Translation2d(-1, -1));
  private static final SwerveModule moduleBR = new SwerveModule(Constants.MotorIds.driveBR, Constants.MotorIds.turnBR, Constants.EncoderIds.turnEncoderBR, new Translation2d(1, -1));

  public SwerveS() {}

  public void drive(double leftJoystickX, double leftJoystickY, double rightJoystickX) {
    // updates each swerve module with latest input from controller
    moduleFL.updateMotors(leftJoystickX, leftJoystickY, rightJoystickX);
    moduleFR.updateMotors(leftJoystickX, leftJoystickY, rightJoystickX);
    moduleBL.updateMotors(leftJoystickY, leftJoystickY, rightJoystickX);
    moduleBR.updateMotors(leftJoystickY, leftJoystickY, rightJoystickX);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
