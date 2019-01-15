package org.usfirst.frc.team4028.robot.subsystems;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
//#endregion

/**
 * This class defines the Chassis Subsystem, it is responsible for:
 * 	- Left & Right drive Motors
 */
public class Chassis extends Subsystem 
{

	private TalonSRX _leftMaster, _leftSlave, _rightMaster, _rightSlave;
	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Chassis _instance = new Chassis();
	
	public static Chassis getInstance() 
	{
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Chassis() 
	{
		_leftMaster = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR);
		_leftSlave = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR);
		_rightMaster = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR);
		
		_leftSlave.follow(_leftMaster);//set slave motors to follow master motors
		_rightSlave.follow(_rightMaster);
		
		_leftMaster.setInverted(true);
		_leftSlave.setInverted(true);
		_rightMaster.setInverted(false);
		_rightSlave.setInverted(false);
        
        configDriveMotors(_leftMaster);
        configDriveMotors(_rightMaster);
        configDriveMotors(_leftSlave);
        configDriveMotors(_rightSlave);
	
	}

	/** Arcade drive with throttle and turn inputs.*/
		
	public void arcadeDrive(double throttleCmdRaw, double turnCmdRaw)
	{
		_leftMaster.set(ControlMode.PercentOutput, throttleCmdRaw + (0.7 * turnCmdRaw));
		_rightMaster.set(ControlMode.PercentOutput, throttleCmdRaw + (0.7 * -1.0 * turnCmdRaw));
	}
	
	private void configDriveMotors(TalonSRX talon) //sets up motors without limit switches and in brake mode
	{
		talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.setNeutralMode(NeutralMode.Brake);
        
	}

	
	public void stop()//Set Motor output to zero
	{
		_leftMaster.set(ControlMode.PercentOutput,0);
		_rightMaster.set(ControlMode.PercentOutput,0);

	}


	@Override
	protected void initDefaultCommand() {}

}