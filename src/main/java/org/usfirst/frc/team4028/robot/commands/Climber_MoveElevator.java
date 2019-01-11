package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_MoveElevator extends Command //Set Elevator to respond to Joystick control
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    Thumbstick _rightThumbstick;
    
    public Climber_MoveElevator(Thumbstick rightThumbstick)
    {
        setInterruptible(true);
        _rightThumbstick = rightThumbstick;
    }
    @Override
    protected void initialize() 
    {
        
    }
    @Override
    protected void execute() //Switch between PID and Percent Output including deadband
    {
        if(Math.abs(_rightThumbstick.getY())>0.05)
        {
            _climber.moveElevator(_rightThumbstick.getY());
        }
        else
        {
            _climber.keepElevatorAtCurrentHeight();
        }

    }
    @Override
    protected boolean isFinished() 
    {
        return false;
    }

}