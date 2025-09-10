// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class MotorIds {
    public static final int driveFL = 10;
    public static final int driveFR = 11;
    public static final int driveBL = 12;
    public static final int driveBR = 13;

    public static final int turnFL = 14;
    public static final int turnFR = 15;
    public static final int turnBL = 16;
    public static final int turnBR = 17;
  }
  public static class EncoderIds { 
    public static final int turnEncoderFL = 20;
    public static final int turnEncoderFR = 21;
    public static final int turnEncoderBL = 22;
    public static final int turnEncoderBR = 23;
  }
}
