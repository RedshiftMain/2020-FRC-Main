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

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Elevator extends SubsystemBase 
{   
    private PosState elevatorState = PosState.Default;

    private final WPI_TalonSRX lElevator = new WPI_TalonSRX(PortConstants.elevator1);
    private final WPI_TalonSRX rElevator = new WPI_TalonSRX(PortConstants.elevator2);

    public Elevator()
    {
        rElevator.follow(lElevator);
        rElevator.setInverted(InvertType.OpposeMaster);
    }

    public void up()
    {
        lElevator.set(ControlMode.PercentOutput, SpeedConstants.elevatorSpeed);
    }

    public void down()
    {
        lElevator.set(ControlMode.PercentOutput, -SpeedConstants.elevatorSpeed);
    }

    @Override
    public void periodic() 
    {
    
    }
}

enum PosState
{
    Default,
}
