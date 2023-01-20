package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int encoderA;
  public final int encoderB;
  public final Rotation2d angleOffset;


  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int encoderA, int encoderB, Rotation2d angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.encoderA = encoderA;
    this.encoderB = encoderB;
    this.angleOffset = angleOffset;
  }
}
