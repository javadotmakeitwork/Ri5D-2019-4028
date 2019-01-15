package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.ElevatorClimber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

public class Climber_PlaceArms extends Command
{
    org.usfirst.frc.team4028.robot.util.BeakXboxController.Trigger _rightTrigger;
    ElevatorClimber _climber = ElevatorClimber.getInstance();
    public Climber_PlaceArms(org.usfirst.frc.team4028.robot.util.BeakXboxController.Trigger righTrigger)
    {
        setInterruptible(true);
        _rightTrigger = righTrigger;
    }
    @Override
    protected void initialize() 
    {
        
    }

    @Override
    protected void execute() 
    {
        _climber.moveArmtoStabilizingPos(_rightTrigger.getX());
    }

    protected boolean isFinished() 
    {
        return false;
    }

}