/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Intake extends SubsystemBase 
{
    private final WPI_VictorSPX intakeVictor = new WPI_VictorSPX(PortConstants.intake);
    private final DoubleSolenoid leftPiston = new DoubleSolenoid(PortConstants.fLeftPiston, PortConstants.rLeftPiston);
    private final DoubleSolenoid rightPiston = new DoubleSolenoid(PortConstants.fRightPiston, PortConstants.rRightPiston);

    public Intake()
    {
        
    }

    public void deploy()
    {
        leftPiston.set(Value.kForward);
        rightPiston.set(Value.kForward);
    }

    public void unploy()
    {
        leftPiston.set(Value.kReverse);
        rightPiston.set(Value.kReverse);
    }

    public void intake()
    {
        intakeVictor.set(ControlMode.PercentOutput, SpeedConstants.intakeSpeed);
    }

    public void outtake()
    {
        intakeVictor.set(ControlMode.PercentOutput, -SpeedConstants.intakeSpeed);
    }

    @Override
    public void periodic() 
    {
    
    }
}
