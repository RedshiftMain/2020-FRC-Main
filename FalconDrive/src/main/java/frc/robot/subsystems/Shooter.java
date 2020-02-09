/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase 
{
  private final WPI_TalonFX lShooter = new WPI_TalonFX(PortConstants.lShooter);
  private final WPI_TalonFX rShooter = new WPI_TalonFX(PortConstants.rShooter);

  private double desiredSpeed;
  private double actualSpeed;
  private double error;
  private double lastError;
  private final double kP = 0.09;
  private final double kD = 0.06;

  public Shooter()
  {
    rShooter.follow(lShooter);
    rShooter.setInverted(InvertType.OpposeMaster);

    //lShooter.configClosedloopRamp(1);

    desiredSpeed = 0;
    actualSpeed = 0;
    error = 0;
    lastError = 0;
  }

  public void shoot(double speed)
  {
    if(speed > 0)
    {
      desiredSpeed = speed;
      error = desiredSpeed - actualSpeed;
      actualSpeed += kP*error - kD*(Math.abs(error-lastError));
      actualSpeed = Math.max(0, actualSpeed);

      lShooter.set(TalonFXControlMode.PercentOutput, actualSpeed);
      int i = 0;
      if(i++ % 10 == 0)
      {
        lastError = error;
      }
    }
    else
    {
      lShooter.set(0);
      desiredSpeed = 0;
      actualSpeed = 0;
      error = 0;
      lastError = 0;
    }
  }

  @Override
  public void periodic() 
  {
  
  }
}
