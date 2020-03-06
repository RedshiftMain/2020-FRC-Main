/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;

public class BetterShooter extends SubsystemBase {
  /**
   * Creates a new BetterShooter because the old one was scuffed
   */
  private final WPI_TalonFX lShooter = new WPI_TalonFX(PortConstants.lShooter);
  private final WPI_TalonFX rShooter = new WPI_TalonFX(PortConstants.rShooter);
  private double speed;
  //units per 100 ms
  private double blehSpeed = 5000;
  private double nearSpeed = 16750;
  private double farSpeed = 19600;

  public BetterShooter() {
    lShooter.configFactoryDefault();
    rShooter.configFactoryDefault();

    lShooter.configNeutralDeadband(.02);
    rShooter.configNeutralDeadband(.02);
    
    rShooter.follow(lShooter);
    rShooter.setInverted(InvertType.OpposeMaster);
    
    lShooter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                          Constants.ShooterConstants.kPIDLoopIdx, 
                                          Constants.ShooterConstants.kTimeoutMs);
    
    lShooter.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
    lShooter.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
    lShooter.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
    lShooter.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);
  
    lShooter.config_kF(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kF, Constants.ShooterConstants.kTimeoutMs);
		lShooter.config_kP(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kP, Constants.ShooterConstants.kTimeoutMs);
		lShooter.config_kI(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kI, Constants.ShooterConstants.kTimeoutMs);
		lShooter.config_kD(Constants.ShooterConstants.kPIDLoopIdx, Constants.ShooterConstants.kGains_Velocit.kD, Constants.ShooterConstants.kTimeoutMs);
  
    lShooter.setNeutralMode(NeutralMode.Coast);
    lShooter.configClosedloopRamp(.25);
    lShooter.setSelectedSensorPosition(0);
    rShooter.setSelectedSensorPosition(0);
  }
  /**
   * shoots slow
   */
  public void shoot(){
    lShooter.set(ControlMode.Velocity, blehSpeed);
    this.speed = blehSpeed;
  }

  /**
   * can shoot with whatever speed you want to
   * @param speed
   */
  public void shoot(double speed){
    lShooter.set(ControlMode.Velocity, speed);
    this.speed = speed;
  }
  /**
   * @apiNote sets shooter motor speed to shoot from trench line 
   */
  public void shootFar(){
    lShooter.set(ControlMode.Velocity, farSpeed);
    this.speed = farSpeed;
  }
  /**
   * @apiNote sets shooter motor speed to shoot from initiation line
   */
  public void shootNear(){
    lShooter.set(ControlMode.Velocity, nearSpeed);
    this.speed = nearSpeed;
  }
  /**
   * sets motor speed to 0
   */
  public void stop(){
    lShooter.set(ControlMode.PercentOutput, 0);
  }

  /**
   * checks if motor is greater than wanted speed
   */
  public boolean atSpeed(){
    System.out.println(lShooter.getSelectedSensorVelocity());
    return lShooter.getSelectedSensorVelocity() >= speed;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Shoot Speed", speed);
  }
}
