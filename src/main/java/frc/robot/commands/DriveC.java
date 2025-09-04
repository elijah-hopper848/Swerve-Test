// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveS;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class DriveC extends Command {
  private final DriveS m_DriveS;

  // private boolean isFinished = false;  

  public DriveC(DriveS subsystem) {
    m_DriveS = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //anyways
    m_DriveS.drive(
      RobotContainer.m_driverController.getLeftY(), 
      RobotContainer.m_driverController.getRightY()
    );
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}