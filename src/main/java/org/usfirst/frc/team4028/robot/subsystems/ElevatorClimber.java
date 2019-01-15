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
    public double _currentPosition=0;
    
    private static ElevatorClimber _instance = new ElevatorClimber();
    public static ElevatorClimber getInstance()
    {
        return _instance;
    }
    private ElevatorClimber()//Constructor For Singleton Pattern
    {
        _elevatorMtr=new TalonSRX(RobotMap.ELEVATOR_MOTOR_CAN_ADDR);
        _climbDriveMtr=new TalonSRX(RobotMap.CLIMB_DRIVE_MOTOR_CAN_ADDR);
        _armMtr = new TalonSRX(RobotMap.STABILIZING_ARM_CAN_ADDR);
        configMtrs();

    }

    public void configMtrs()//Set Up All Encoders, Limit Switches, Sensor Phases, and PID Constants for the Motors
    {
        _elevatorMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
        _elevatorMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        _elevatorMtr.setNeutralMode(NeutralMode.Brake);
        _elevatorMtr.setSensorPhase(true);
        _elevatorMtr.config_kP(0, 1.6, 10);
        _elevatorMtr.config_kI(0, 0, 10);
        _elevatorMtr.config_kD(0, 16, 10);
        _elevatorMtr.config_kF(0, 1.0, 10);
        _climbDriveMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 10);
        _climbDriveMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 10);
        _armMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 10);
        _armMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 10);
    }


    public void moveElevator(double throttleCmd)//Control Elevator with a Throttle Command from the Joystick
    {
        if(throttleCmd<0)
        {
            _elevatorMtr.set(ControlMode.PercentOutput, throttleCmd*0.8);
        }
        else
        {
            _elevatorMtr.set(ControlMode.PercentOutput, throttleCmd*0.5);
        }
        _currentPosition=_elevatorMtr.getSelectedSensorPosition(0);
        
    }

    public void moveArmtoStabilizingPos(double throttle)//Moves the Stabilizing Arm into Position to Climb
    {
        _armMtr.set(ControlMode.PercentOutput, 0.4*throttle);
    }

    public void stopArms()
    {
        _armMtr.set(ControlMode.PercentOutput, 0);
    }

    public void setElevatorPos(int pos)
    {
        _elevatorMtr.setSelectedSensorPosition(pos, 0, 10);
    }

    public void keepElevatorAtCurrentHeight()//Utilize Motion Magic to Prevent the Elevator From Backdriving
    {
        _elevatorMtr.set(ControlMode.MotionMagic, _currentPosition);
    }

 

    public void postElevateDrive(double throttleCmd)//Drive the Robot Forward if the Elevator Is Up
    {
        
        _climbDriveMtr.set(ControlMode.PercentOutput, throttleCmd);

    }

    public void stopElevatorDrive()//Stop the Driving of the Robot with the Robot Elevated
    {
        _climbDriveMtr.set(ControlMode.PercentOutput, 0);
    }
}