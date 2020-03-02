/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class BetterElevator extends SubsystemBase 
{   
    /*private final WPI_TalonSRX lElevator = new WPI_TalonSRX(PortConstants.lElevator);
    private final WPI_TalonSRX rElevator = new WPI_TalonSRX(PortConstants.rElevator);
    private final DoubleSolenoid brakePiston = new DoubleSolenoid(PortConstants.elPiston, PortConstants.erPiston);

    public BetterElevator()
    {
        //check direction
        rElevator.follow(lElevator);
        rElevator.setInverted(InvertType.OpposeMaster);
    }
    public void move()
    {
        if(-1*RobotContainer.stick2.getRawAxis(1) > 0)
            lElevator.set(ControlMode.PercentOutput, ElevatorConstants.elevatorClimbSpeed*RobotContainer.stick2.getRawAxis(1));
        else
            lElevator.set(ControlMode.PercentOutput, ElevatorConstants.elevatorHookSpeed*RobotContainer.stick2.getRawAxis(1));
    }
    public void up()
    {
        lElevator.set(ControlMode.PercentOutput, SpeedConstants.elevatorSpeed);
    }

    public void down()
    {
        lElevator.set(ControlMode.PercentOutput, -SpeedConstants.elevatorSpeed);
    }
    
    public void brakesForward()
    {
        ePiston.set(Value.kForward);
    }

    public void brakesReverse()
    {
        ePiston.set(Value.kReverse);
    }
 
    public void stop()
    {
        lElevator.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() 
    {
    
    }*/
}
