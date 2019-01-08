package org.usfirst.frc.team4028.robot.subsystems;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.RobotMap;
import org.usfirst.frc.team4028.robot.auton.RobotState;
import org.usfirst.frc.team4028.robot.auton.control.Path;
import org.usfirst.frc.team4028.robot.auton.control.PathFollower;
import org.usfirst.frc.team4028.robot.auton.motion.RigidTransform;
import org.usfirst.frc.team4028.robot.auton.motion.Rotation;
import org.usfirst.frc.team4028.robot.auton.motion.Twist;
import org.usfirst.frc.team4028.robot.auton.util.Kinematics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.util.GeneralUtilities;
import org.usfirst.frc.team4028.robot.util.LogDataBE;
//#endregion

/**
 * This class defines the Chassis Subsystem, it is responsible for:
 * 	- Left & Right drive Motors
 *  - Solenoid that controls the shifting
 */
public class Chassis extends Subsystem 
{

	private TalonSRX _leftMaster, _leftSlave, _rightMaster, _rightSlave;
	private DoubleSolenoid _shifter;
	
	private NavXGyro _navX = NavXGyro.getInstance();
	
	public static final double ENCODER_COUNTS_PER_WHEEL_REV = 30725.425;		// account for gear boxes

	public double _leftMtrDriveSetDistanceCmd;
	public double _rightMtrDriveSetDistanceCmd;
	//public double _throttle;
	private double _targetAngle, _angleError;
	private boolean _isTurnRight;
	private static final double ENCODER_ROTATIONS_PER_DEGREE = 46.15/3600;
	private RobotState _robotState = RobotState.getInstance();
	private double _leftMasterVelocityLoggingLastLogTime;
	private double _leftMasterVelocityLoggingThisTime;
	private double _leftMasterPreviousVelocity = 0;
	private double _leftMasterCurrentVelocity = 0;
	private boolean _isFirstTimeLoggingAccel = true;

	public enum ChassisState
	{
		UNKNOWN,
		PERCENT_VBUS,
		AUTO_TURN, 
		FOLLOW_PATH,
		DRIVE_SET_DISTANCE
	}
	/*
	private static final double[] MOTION_MAGIC_TURN_PIDF_GAINS = {0.25, 0.0, 30.0, 0.095};
	private static final double[] MOTION_MAGIC_STRAIGHT_PIDF_GAINS = {0.15, 0.0, 20.0, 0.095};
	private static final double[] LOW_GEAR_VELOCITY_PIDF_GAINS = {0.15, 0.0, 1.5, 0.085}; 
	private static final double[] HIGH_GEAR_VELOCITY_PIDF_GAINS = {0.09, 0.0, 1.3, 0.044}; 
    
    private static final int[] MOTION_MAGIC_TURN_VEL_ACC = {80 * 150, 170 * 150};
	private static final int[] MOTION_MAGIC_STRAIGHT_VEL_ACC = {80 * 150, 170 * 150};
	*/
	ChassisState _chassisState = ChassisState.UNKNOWN;
	Path _currentPath;
	PathFollower _pathFollower;
	double _leftEncoderPrevDistance, _rightEncoderPrevDistance = 0;
	double _centerTargetVelocity, _leftTargetVelocity, _rightTargetVelocity;
	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Chassis _instance = new Chassis();
	
	public static Chassis getInstance() 
	{
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Chassis() {
		_leftMaster = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR);
		_leftSlave = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR);
		_rightMaster = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);
		_rightSlave = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR);
		
		_leftSlave.follow(_leftMaster);
		_rightSlave.follow(_rightMaster);
		
		_leftMaster.setInverted(true);
		_leftSlave.setInverted(true);
		_rightMaster.setInverted(false);
		_rightSlave.setInverted(false);
		
		configMasterMotors(_leftMaster);
		configMasterMotors(_rightMaster);
        
        configDriveMotors(_leftMaster);
        configDriveMotors(_rightMaster);
        configDriveMotors(_leftSlave);
        configDriveMotors(_rightSlave);

		_shifter = new DoubleSolenoid(RobotMap.PCM_CAN_ADDR, RobotMap.SHIFTER_EXTEND_PCM_PORT, RobotMap.SHIFTER_RETRACT_PCM_PORT);
		
	}


	public void updateChassis(double timestamp){
		switch(_chassisState) {
			case UNKNOWN:
			return;
			case PERCENT_VBUS:
				return;
				
				
			case DRIVE_SET_DISTANCE:
				//GeneralUtilities.setPIDFGains(_leftMaster, MOTION_MAGIC_STRAIGHT_PIDF_GAINS);
				//GeneralUtilities.setPIDFGains(_rightMaster, MOTION_MAGIC_STRAIGHT_PIDF_GAINS);
				moveToTargetPosDriveSetDistance();
				return;
				
			case FOLLOW_PATH:
				if (get_isHighGear()) {
				//	GeneralUtilities.setPIDFGains(_leftMaster, HIGH_GEAR_VELOCITY_PIDF_GAINS);
				//	GeneralUtilities.setPIDFGains(_rightMaster, HIGH_GEAR_VELOCITY_PIDF_GAINS);
				} else {
				//	GeneralUtilities.setPIDFGains(_leftMaster, LOW_GEAR_VELOCITY_PIDF_GAINS);
				//	GeneralUtilities.setPIDFGains(_rightMaster, LOW_GEAR_VELOCITY_PIDF_GAINS);
				}
				
	
		}
	}



	
	/* ===== Chassis State: PERCENT VBUS ===== */
	/** Arcade drive with throttle and turn inputs. Includes anti-tipping. */
		
		public void arcadeDrive(double throttleCmdRaw, double turnCmdRaw)
  		{
    		_leftMaster.set(ControlMode.PercentOutput, throttleCmdRaw + (0.7 * turnCmdRaw));
    		_rightMaster.set(ControlMode.PercentOutput, throttleCmdRaw + (0.7 * -1.0 * turnCmdRaw));
  		}
	
	
	/* ===== SHIFTER ===== */
    public synchronized void toggleShifter() {
    	setHighGear(!get_isHighGear());	// Inverse of current solenoid state
    }
	
	public synchronized void setHighGear(boolean isHighGear) {
		if (isHighGear) 
			_shifter.set(Constants.SHIFTER_HIGH_GEAR_POS);
		else 
			_shifter.set(Constants.SHIFTER_LOW_GEAR_POS);
	}
	
	private void configMasterMotors(TalonSRX talon) {
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
	}
	
	private void configDriveMotors(TalonSRX talon) {
		talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
        
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.



	public void moveToTargetPosDriveSetDistance ()
	{
		setLeftRightCommand(ControlMode.MotionMagic, _leftMtrDriveSetDistanceCmd, _rightMtrDriveSetDistanceCmd);
	}
	
	public void stop()
	{
		setLeftRightCommand(ControlMode.PercentOutput, 0, 0);
		setHighGear(false);

	}




	//=====================================================================================
	// Property Accessors
	//=====================================================================================
	public double get_leftSpeed() {
		return _leftMaster.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}
	
	public double get_rightSpeed() {
		return -_rightMaster.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}

    public double get_leftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(get_leftSpeed());
    }

    public double get_rightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(get_rightSpeed());
	}
	

	private synchronized boolean get_isHighGear() {
		return _shifter.get() == Constants.SHIFTER_HIGH_GEAR_POS;
	}

	public double get_leftPos()
	{
		return _leftMaster.getSelectedSensorPosition(0);
	}
	public double get_rightPos()
	{
		return _rightMaster.getSelectedSensorPosition(0);
	}
	
	public double get_Heading() {
		return _navX.getYaw();
	}

	//=====================================================================================
	// Private Helper methods below
	//=====================================================================================
	private void setLeftRightCommand(ControlMode mode, double leftCommand, double rightCommand) {
		_leftMaster.set(mode, leftCommand);
		_rightMaster.set(mode, rightCommand);
	}
	   
    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }
    
    private static double rotationsToInches(double rot) {
        return rot * (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI);
    } 

	private static double InchestoNU (double inches){
		return inches * ENCODER_COUNTS_PER_WHEEL_REV/(Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI);
	}
	private static double NUtoInches (double NU)
	{
		return NU *Constants.DRIVE_WHEEL_DIAMETER_IN*Math.PI / ENCODER_COUNTS_PER_WHEEL_REV;
	}

	public static double inchesPerSecToNU(double inches_per_second) 
	{
        return inches_per_second * ENCODER_COUNTS_PER_WHEEL_REV / (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI * 10);
	}

	public static double NUper100msToInchesPerSec(double NU_per_100ms){
		return NU_per_100ms*10*Constants.DRIVE_WHEEL_DIAMETER_IN*Math.PI/(ENCODER_COUNTS_PER_WHEEL_REV);
	}
	public double getLeftSpeedRPM() {
		return _leftMaster.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}
	
	public double getRightSpeedRPM() {
		return -_rightMaster.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}
	public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(getLeftSpeedRPM());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(getRightSpeedRPM());


	}
	
	

	//=====================================================================================
	// Support Methods
	//=====================================================================================
	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("Left Actual Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(get_leftVelocityInchesPerSec(), 2)));
		//logData.AddData("Left Target Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(_leftTargetVelocity, 2)));
		logData.AddData("Left Output Current", String.valueOf(GeneralUtilities.roundDouble(_leftMaster.getOutputCurrent(), 2)));
		
		logData.AddData("Right Actual Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(-get_rightVelocityInchesPerSec(), 2)));
		//logData.AddData("Right Target Velocity [in/s]", String.valueOf(GeneralUtilities.roundDouble(_rightTargetVelocity, 2)));
		logData.AddData("Right Output Current", String.valueOf(GeneralUtilities.roundDouble(_rightMaster.getOutputCurrent(), 2)));

		//logData.AddData("Pose X", String.valueOf(RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x()));
		//logData.AddData("Pose Y", String.valueOf(RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y()));
		//logData.AddData("Pose Angle", String.valueOf(RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees()));
		//logData.AddData("Remaining Distance", String.valueOf(getRemainingPathDistance()));
		
		//logData.AddData("Center Target Velocity", String.valueOf(GeneralUtilities.roundDouble(_centerTargetVelocity, 2)));

	}


	
	public void updateDashboard() 
	{
		SmartDashboard.putBoolean("IsHighGear", get_isHighGear());
		SmartDashboard.putNumber("Chassis: Left Velocity", GeneralUtilities.roundDouble(get_leftVelocityInchesPerSec(), 2));
		SmartDashboard.putNumber("Chassis: Right Velocity", GeneralUtilities.roundDouble(get_rightVelocityInchesPerSec(), 2));
		
		SmartDashboard.putNumber("Chassis: Left Wheel Target Velocity", -1.0); //GeneralUtilities.roundDouble(_leftTargetVelocity, 2));
		SmartDashboard.putNumber("Chasiss: Right Wheel Target Velocity",  -1.0); // GeneralUtilities.roundDouble(_leftTargetVelocity, 2));
		
		SmartDashboard.putNumber("Chassis: Angle", GeneralUtilities.roundDouble(get_Heading(), 2));
		SmartDashboard.putString("Chassis: Robot Pose", "N/A"); //RobotState.getInstance().getLatestFieldToVehicle().getValue().toString());
	}

	@Override
	protected void initDefaultCommand() {

	}

}