// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;


import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {


  private final Swerve s_Swerve = new Swerve();
  private Joystick joystick;

  @Override
  public void robotInit(){
    joystick = new Joystick(0);
  }

  
  @Override
  public void teleopPeriodic() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -joystick.getY(), 
            () -> -joystick.getX(), 
            () -> -joystick.getTwist(), 
            () -> joystick.getTrigger()
        )
    );
  }
}
