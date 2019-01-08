package org.usfirst.frc.team4028.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.RobotMap;
import java.awt.datatransfer.FlavorMap;

import edu.wpi.first.wpilibj.DigitalInput;
public class ElevatorClimber
{
    private TalonSRX _elevatorMtr,_climbDriveMtr, _armMtr;
    private boolean _hasElevatorBeenZeroed, _isRobotElevated,_readyToElevate =false;
    private DigitalInput _isElevatorOnGround;
    
    
    private static ElevatorClimber _instance = new ElevatorClimber();
    public static ElevatorClimber getInstance()
    {
        return _instance;
    }
    private ElevatorClimber()
    {
        _elevatorMtr=new TalonSRX(RobotMap.ELEVATOR_MOTOR_CAN_ADDR);
        _climbDriveMtr=new TalonSRX(RobotMap.CLIMB_DRIVE_MOTOR_CAN_ADDR);
        _armMtr = new TalonSRX(RobotMap.RANDOM_ARM_THINGY_CAN_ADDR);
        _isElevatorOnGround = new DigitalInput(RobotMap.ELEVATOR_ON_GROUND_LIMIT_SWITCH_DIO_PORT);
        configMtrs();

    }

    public void configMtrs()
    {
        _elevatorMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        _elevatorMtr.setNeutralMode(NeutralMode.Brake);
        _armMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.config_kP(0, 0, 10);
        _elevatorMtr.config_kI(0, 0, 10);
        _elevatorMtr.config_kD(0, 0, 10);
        _elevatorMtr.config_kF(0, 0, 10);
    }

    public void zeroElevator()
    {
        if(_elevatorMtr.getSensorCollection().isRevLimitSwitchClosed())
        {
            _elevatorMtr.set(ControlMode.PercentOutput, -0.1);
            _hasElevatorBeenZeroed=false;
        }
        else
        {
            _elevatorMtr.set(ControlMode.PercentOutput, 0);
            _elevatorMtr.setSelectedSensorPosition(0, 0, 10);
            _hasElevatorBeenZeroed=true;
        }
    }

    public void elevate()
    {
        if(_readyToElevate)
        {
            if(_elevatorMtr.getSensorCollection().isFwdLimitSwitchClosed())
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
        else
        {
            yeetTheArm();
        }
    }

    public void yeetTheArm()
    {
        if(_armMtr.getSensorCollection().isFwdLimitSwitchClosed())
        {
            _armMtr.set(ControlMode.PercentOutput, 0.3);
            _readyToElevate=false;
        }
        else
        {
            _armMtr.set(ControlMode.PercentOutput, 0);
            _readyToElevate=true;
        } 
    }

    public void keepElevatorAtMaxHeight()
    {
        _elevatorMtr.set(ControlMode.MotionMagic, _elevatorMtr.getSelectedSensorPosition(0));
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
        if(_isElevatorOnGround.get())
        {
            _climbDriveMtr.set(ControlMode.PercentOutput, 0.1);
            _elevatorMtr.set(ControlMode.PercentOutput, -0.1);
        }
        else
        {
            _elevatorMtr.set(ControlMode.PercentOutput, 0.1);
        }
    }

}