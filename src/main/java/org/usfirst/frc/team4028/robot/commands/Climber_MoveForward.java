package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;
import org.usfirst.frc.team4028.robot.util.BeakXboxController;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

public class Climber_MoveForward extends Command
{
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    Button _button;
    public Climber_MoveForward(Button button)
    {
        _button=button;
        setInterruptible(true);
    }
    @Override
    protected void initialize() 
    {
        
    }
    @Override
    protected void execute() 
    {
        if(_button.get())
        {
            _climber.postElevateDrive();
        }
        else
        {
            _climber.stopElevatorDrive();
        }
    }
    
    protected boolean isFinished() 
    {
        return false;
    }

}