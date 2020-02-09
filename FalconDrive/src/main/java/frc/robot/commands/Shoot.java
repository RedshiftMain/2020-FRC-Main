/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Shooter shooter;
  private final double speed;

  public Shoot(Shooter shoot, double s) 
  {
    shooter = shoot;
    speed = s;
    addRequirements(shooter);
  }

  @Override
  public void initialize() 
  {
      
  }

  @Override
  public void execute() 
  {
    shooter.shoot(speed);
  }

  @Override
  public void end(boolean interrupted) 
  {
    shooter.shoot(0);
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
