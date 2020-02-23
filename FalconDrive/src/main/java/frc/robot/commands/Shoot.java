/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;

  private double ta;
  private double tv;
  private double speed;

  public Shoot(Shooter shoot) 
  {
    shooter = shoot;
    addRequirements(shooter);
  }

  @Override
  public void initialize() 
  {
    speed = 0;
  }

  @Override
  public void execute() 
  {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if(tv < 1.0)
    {
        shooter.stop();
    }
    else
    {
        speed = SpeedConstants.minShootSpeed + (SpeedConstants.maxShootSpeed - SpeedConstants.minShootSpeed) / (SpeedConstants.maxArea - SpeedConstants.minArea) * ta;
        if(speed > SpeedConstants.maxShootSpeed)
        {
          speed = SpeedConstants.maxShootSpeed;
        }
        if(speed < SpeedConstants.minShootSpeed)
        {
          speed = SpeedConstants.minShootSpeed;
        }

        shooter.shootSpeed(speed);
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    shooter.stop();
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
