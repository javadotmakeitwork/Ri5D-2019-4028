package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;

import edu.wpi.first.wpilibj.command.Command;

public class Climber_MoveElevator extends Command
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
    protected void execute() 
    {
        _climber.moveElevator(_rightThumbstick.getY());
    }
    @Override
    protected boolean isFinished() 
    {
        return false;
    }

}