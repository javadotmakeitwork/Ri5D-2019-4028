package org.usfirst.frc.team4028.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.RobotMap;
public class ElevatorClimber
{
    private TalonSRX _elevatorMtr,_climbDriveMtr;
    private boolean _hasElevatorBeenZeroed, _isRobotElevated;
    
    
    private static ElevatorClimber _instance = new ElevatorClimber();
    public static ElevatorClimber getInstance()
    {
        return _instance;
    }
    private ElevatorClimber()
    {
        _elevatorMtr=new TalonSRX(RobotMap.ELEVATOR_MOTOR_CAN_ADDR);
        _climbDriveMtr=new TalonSRX(RobotMap.CLIMB_DRIVE_MOTOR_CAN_ADDR);
        configElevatorMotor();

    }

    public void configElevatorMotor()
    {
        _elevatorMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.setNeutralMode(NeutralMode.Brake);

        /*
        LIMIT SWITCH or PID Configuration Here
        */
    }

    public void zeroElevator()
    {
        if(!_elevatorMtr.getSensorCollection().isRevLimitSwitchClosed())
        {
            _elevatorMtr.set(ControlMode.PercentOutput, -0.1);
            _hasElevatorBeenZeroed=false;
        }
        else
        {
            _elevatorMtr.set(ControlMode.PercentOutput, 0);
            _hasElevatorBeenZeroed=true;
        }
    }

    public void elevate()
    {
        if(!_elevatorMtr.getSensorCollection().isFwdLimitSwitchClosed())
        {
            _elevatorMtr.set(ControlMode.PercentOutput, 0.1);
            _isRobotElevated = false;
        }
        else
        {
            _elevatorMtr.set(ControlMode.PercentOutput, 0);
            _isRobotElevated=true;

        } 
    }

    public boolean hasElevatorBeenZeroed()
    {
        return _hasElevatorBeenZeroed;
    }

    public boolean isRobotElevated()
    {
        return _isRobotElevated;
    }

    public void postElevateDrive()
    {
        if(_isRobotElevated)
        {
            _climbDriveMtr.set(ControlMode.PercentOutput, 0.1);
        }
        else
        {
            _climbDriveMtr.set(ControlMode.PercentOutput,0.0);
        }

    }

    public void stopElevatorDrive()
    {
        _climbDriveMtr.set(ControlMode.PercentOutput, 0);
    }

    public void driveDown()
    {
      //#TODO  
    }

}