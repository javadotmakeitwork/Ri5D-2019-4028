package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Chassis_ArcadeDriveAction extends Command
{
    private Chassis _chassis = Chassis.getInstance();
    
    double _waitTime;
    double _throttle;
    double _turn = 0; //Intitialized to 0 so that if constructor has no turn it will turn
    double _startTime; 
    private Thumbstick _leftThumbstick;
    private Thumbstick _rightThumbstick;
    boolean _isDriverControlled;


    public Chassis_ArcadeDriveAction(double throttle, double waitTime)
    {
        _throttle = throttle;
        _waitTime = waitTime;
        _isDriverControlled=false;
    }

    public Chassis_ArcadeDriveAction(double throttle, double waitTime, double turn)
    {
        _throttle = throttle;
        _waitTime = waitTime;
        _turn = turn;
        _isDriverControlled=false;
    }


    public Chassis_ArcadeDriveAction(Thumbstick leftStick, Thumbstick rightStick) 
    {
        setInterruptible(true);
        _leftThumbstick = leftStick;
        _rightThumbstick = rightStick;
        _isDriverControlled=true;
	}

    protected void initialize() 
    {    	
        _startTime = Timer.getFPGATimestamp();
		
    }

    
    protected void execute() 
    {
        if(!_isDriverControlled)
        {
            if ((Timer.getFPGATimestamp() - _startTime) > _waitTime) 
            {
                _chassis.stop();
            } 
            else 
            {
                _chassis.arcadeDrive(-_throttle, _turn);
            }
        }
        else
        {
            _chassis.arcadeDrive(-_leftThumbstick.getY(), _rightThumbstick.getX());
        }
    }

    protected boolean isFinished() {
        if(_isDriverControlled)
        {
            return false;
        }
        else
        {
            return (Timer.getFPGATimestamp() - _startTime) > _waitTime;
        }
    }

    protected void end() {
        _chassis.stop();
    }


    protected void interrupted() {
    }


}