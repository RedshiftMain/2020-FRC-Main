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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Shooter extends SubsystemBase 
{
  private final WPI_TalonFX lShooter = new WPI_TalonFX(PortConstants.lShooter);
  private final WPI_TalonFX rShooter = new WPI_TalonFX(PortConstants.rShooter);

  private double lastSpeed;

  public Shooter()
  {
    rShooter.follow(lShooter);
    rShooter.setInverted(InvertType.OpposeMaster);

    lShooter.configOpenloopRamp(SpeedConstants.rampSpeed);
    lShooter.setNeutralMode(NeutralMode.Brake);
  }

  public void shoot()
  {
    lShooter.set(ControlMode.PercentOutput, SpeedConstants.minShootSpeed);
  }

  public void shootSpeed(double speed)
  {
    lShooter.set(ControlMode.PercentOutput, speed);
  }

  public void shootFast()
  {
    lShooter.set(ControlMode.PercentOutput, SpeedConstants.maxShootSpeed);
  }

  public void stop()
  {
    lShooter.set(ControlMode.PercentOutput, 0);
  }

  //needs to be tested
  public boolean atSpeed()
  {
    double currentSpeed = lShooter.getSelectedSensorVelocity();

    return currentSpeed <= lastSpeed;
    //return lShooter.getSelectedSensorVelocity() > 16000;
  }

  @Override
  public void periodic() 
  {
  
  }
}
