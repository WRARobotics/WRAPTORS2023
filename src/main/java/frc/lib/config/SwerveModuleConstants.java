package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int encoderPWMChannel;
  public final Rotation2d angleOffset;


  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int encoderPWMChannel, Rotation2d angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.encoderPWMChannel = encoderPWMChannel;
    this.angleOffset = angleOffset;
  }
}
