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
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootForTime extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private final Magazine magazine;
  private final Feeder feeder;
  private final Timer timer = new Timer();
  private final double endTime;

  public ShootForTime(double time, Shooter shoot, Magazine magazine, Feeder feeder) 
  {
    shooter = shoot;
    this.magazine = magazine;
    this.feeder = feeder;
    endTime = time;
    addRequirements(shooter, magazine, feeder);
  }

  @Override
  public void initialize() 
  {
    shooter.shoot();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() 
  {
    if(shooter.atSpeed())
    {
      magazine.load();
      feeder.feed();
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    shooter.stop();
    magazine.stop();
    feeder.stop();
  }

  @Override
  public boolean isFinished() 
  {
    return timer.get() > endTime;
  }
}
