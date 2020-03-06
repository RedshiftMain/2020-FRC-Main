/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.SpeedConstants;

public class Elevator extends SubsystemBase 
{   
    //private PosState elevatorState = PosState.Default;

    //private final WPI_TalonSRX lElevator = new WPI_TalonSRX(PortConstants.elevator1);
    //private final WPI_TalonSRX rElevator = new WPI_TalonSRX(PortConstants.elevator2);
    private final WPI_TalonFX lElevator = new WPI_TalonFX(PortConstants.elevator1);
    private final WPI_TalonFX rElevator = new WPI_TalonFX(PortConstants.elevator2);
    //private final DoubleSolenoid ePiston = new DoubleSolenoid(PortConstants.elPiston, PortConstants.erPiston);

    /**
     * constructor for elevator class
     */
    public Elevator()
    {
        rElevator.follow(lElevator);
        rElevator.setInverted(InvertType.OpposeMaster);
        lElevator.setNeutralMode(NeutralMode.Brake);
        rElevator.setNeutralMode(NeutralMode.Brake);
    }
    public void move()
    {
        if(RobotContainer.stick2.getRawAxis(1) > 0)
            lElevator.set(ControlMode.PercentOutput, ElevatorConstants.elevatorClimbSpeed*RobotContainer.stick2.getRawAxis(1));
        else
            lElevator.set(ControlMode.PercentOutput, ElevatorConstants.elevatorHookSpeed*RobotContainer.stick2.getRawAxis(1));
    }
    /**
     * @apiNote makes elevator go up
     */
    public void up()
    {
        lElevator.set(ControlMode.PercentOutput, SpeedConstants.elevatorSpeed);
    }
    /**
     * @apiNote makes elevator go down
     */
    public void down()
    {
        lElevator.set(ControlMode.PercentOutput, -SpeedConstants.elevatorSpeed);
    }
    
    public void brakesForward()
    {
        //ePiston.set(Value.kForward);
    }

    public void brakesReverse()
    {
        //ePiston.set(Value.kReverse);
    }
    /**
     * @apiNote stops motor by setting its speed to 0
     */
    public void stop()
    {
        lElevator.set(ControlMode.PercentOutput, 0);
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
