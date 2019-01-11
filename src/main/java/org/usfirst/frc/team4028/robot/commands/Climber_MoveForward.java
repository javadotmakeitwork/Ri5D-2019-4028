package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;
import edu.wpi.first.wpilibj.command.Command;

public class Climber_MoveForward extends Command //Set Elevator to respond to Joystick control via command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    Thumbstick _leftThumbstick;
    public Climber_MoveForward(Thumbstick leftThumbstick)
    {
        _leftThumbstick=leftThumbstick;
        setInterruptible(true);
    }
    @Override
    protected void initialize() 
    {
        
    }
    @Override
    protected void execute() //Set Elevator Motor wheels to drive after Elevator is extended
    {
        _climber.postElevateDrive(_leftThumbstick.getY());
    }
    
    protected boolean isFinished() 
    {
        return false;
    }

}