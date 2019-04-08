/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//Import needed classes
import com.ctre.phoenix.motorcontrol.ControlMode;	// Import classes from CTRE
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.CameraServer;	// Import WPILib classes
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
	// Create objects
	private DifferentialDrive driveTrain;	// Drive train
	// Talons
	private TalonSRX verticalArm;	// Vertical motor
	private TalonSRX gripperL;	// Gripper Motors
	private TalonSRX gripperR;
	// Controllers
	private Joystick driveStick;	// Joystick
	private Joystick armGamepad;	// Gamepad
	
	private Timer robotTimer;	// Timer
	// Solenoids
	private DoubleSolenoid gripperSolenoid;	// Gripper solenoid
	private Solenoid armSolenoid;	// Arm rocker solenoid
	// Digital Inputs (jumpers, set robot position)
	private DigitalInput jumpA;	// Position jumper a
	private DigitalInput jumpB;	// Position jumper b

	// Create variables to store joystick and limit switches values
	boolean button1Pressed = false;	// Spinny bois spin in, failsafe false
	boolean button2Pressed = false;	// Spinny bois spin out, failsafe false
	boolean button3Pressed = false;	// Arm down, failsafe false
	boolean button4Pressed = false;	// Arm open, failsafe false
	boolean button5Pressed = false;	// Arm up, failsafe false
	boolean button6Pressed = false;	// Arm close, failsafe false
	boolean button12Pressed = false;	// Button 12, failsafe false
	boolean RB = false;				// Spinny bois spin in, failsafe false
	boolean LB = false;				// Spinny bois spin out, failsafe false
	boolean jumperA = false;		// Position jumper A, failsafe false
	boolean jumperB = false;		// Position jumper B, failsafe false
	
	double speed = 0.5;		// Drive speed multiplier, initial 0.5
	double driveY = 0.0;	// Drive y (f/b), initial 0
	double driveX = 0.0;	// Drive y (rot), initial 0

	int robotLocation = 0;	// Robot location, failsafe 0 (go nowhere)

	String gameData = "";	// Game data, failsafe empty

	@Override
	public void robotInit() {	// Run when robot is turned on. Initialize m.controllers, etc.
		// Initialize drive train
		Spark m_rearLeft = new Spark(0);	// Initialize left Sparks
		Spark m_frontLeft = new Spark(1);

		SpeedControllerGroup m_left = new SpeedControllerGroup(m_rearLeft, m_frontLeft);	// Join left Sparks into a group

		Spark m_frontRight = new Spark(2);	// Initialize right Sparks
		Spark m_rearRight = new Spark(3);
		SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);	// Join right Sparks into a group

		driveTrain = new DifferentialDrive(m_left, m_right);	// Create drive train with Spark groups
		
		// Initialize arm
		verticalArm = new TalonSRX(0);	// Initialize vertical arm motor
		gripperL = new TalonSRX(1);	// Initialize gripper Talons
		gripperR = new TalonSRX(2);
		
		// Initialize controllers
		driveStick = new Joystick(0);	// Initialize joystick
		armGamepad = new Joystick(1);	// Initialize gamepad

		robotTimer = new Timer();	// Initialize timer
		
		// Initialize solenoids
		gripperSolenoid = new DoubleSolenoid(0,1);	// Initialize gripper solenoids
		armSolenoid = new Solenoid(2);	// Initialize arm rocker solenoid
		
		jumpA = new DigitalInput(2);	// Jumpers:
		jumpB = new DigitalInput(3);	// FS: XX	1: A	2: AB	3: B

		// Find new camera server code!!!  NP 1/22/2019
		// CameraServer.getInstance().startAutomaticCapture();	// Start camera stream to DS
	}
	
	@Override
	public void autonomousInit() {	// Initial autonomous code (run once at beginning of autonomous)
		gameData = DriverStation.getInstance().getGameSpecificMessage();	// Get game data (switch/scale positions)
		jumperA = !jumpA.get();	// Get Value of Position jumpers
		jumperB = !jumpB.get();	//  (false is true, so a jumper plugged in reads false, but is inverted to true by !)

		// Set robotLocation based on jumpers
		if (jumperA && !jumperB) {	// 2
			robotLocation = 1;
		} else if (jumperA && jumperB) {	// 2 & 3
			robotLocation = 2;
		} else if (!jumperA && jumperB) {	// 3
			robotLocation = 3;
		} else {	// None
			robotLocation = 0;	// Failsafe
		}

		robotTimer.reset();	// Set timer to 0
		robotTimer.start();	// and start it
	}

	@Override
	public void autonomousPeriodic() {	// Periodic autonomous code (run every (10ms?) while autonomous is active)

		if (robotTimer.get() > 1.0 && robotTimer.get() < 2.0) {	// Deploy arm for 0.3 seconds
			armSolenoid.set(true);
		}

		if(gameData.length() > 0 &&  gameData.charAt(0) == 'L') {	// If our switch is to the left...
    		switch (robotLocation) {
    			case 0:	// ...and we do not know where we are...
    				driveTrain.arcadeDrive(0.0, 0.0);	// ...don't drive
    			break;
    			
    			case 1:	// Left Switch ...and we are to the left...
    				if (robotTimer.get() > 1.3 && robotTimer.get() < 4.0) {	// ...and the time is between 0.3 and 1 second(s)...
    					driveTrain.arcadeDrive(0.7, 0.0);	// ...drive straight at 0.7 speed
    					
    				} else if (robotTimer.get() > 4 && robotTimer.get() < 4.9) {	// ...and the time is between 4 and 5.3 seconds...
    					driveTrain.arcadeDrive(0.0, 0.6);
    					
    				} else if (robotTimer.get() > 5 && robotTimer.get() < 6) {
    					driveTrain.arcadeDrive(0.6, 0.0);
    					
    				} else if (robotTimer.get() > 6 && robotTimer.get() < 6.1) {	// ... and the time is between 5.3 and 5.4 seconds...
    						gripperSolenoid.set(DoubleSolenoid.Value.kForward);	// ...drop the cube
    						
    				} else {	// ...and is none of the above (more than 5.4 seconds)...
    					driveTrain.arcadeDrive(0.0, 0.0);	// ...stop
    				}
    				
    			break;
    			
    			case 2:	// Left Switch  ...and we are in the middle...
    				if (robotTimer.get() > 0.5 && robotTimer.get() < 1.2) {	// ...and the time is between 0.3 and 1 second(s)...
    					driveTrain.arcadeDrive(0.6, 0.0);	// ...drive straight at 0.7 speed
    					
    				} else if (robotTimer.get() > 2.4 && robotTimer.get() < 4.0) {	// ...and the time is between 1.4 and 4 seconds...
    					driveTrain.arcadeDrive(0.6, 0.0);	// ...drive straight at 0.7 speed
    					
    				} else if (robotTimer.get() > 5.4 && robotTimer.get() < 6.8) {	// ... and the time is between 5.3 and 5.4 seconds...
    					driveTrain.arcadeDrive(0.7, 0.0);
    					
    				} else if (robotTimer.get() > 6.8 && robotTimer.get() < 7.0) {
    					gripperSolenoid.set(DoubleSolenoid.Value.kForward);	// ...drop the cube
    					
    				} else {	// ...and is none of the above (more than 5.4 seconds)...
    					driveTrain.arcadeDrive(0.0, 0.0);	// ...stop
    					
    				}
    			break;
    			
    			case 3:  // Left Switch ... and we are to the right...

    				if (robotTimer.get() > 1.3 && robotTimer.get() < 4) {	// ...and the time is between 0.3 and 1 second(s)...
    					driveTrain.arcadeDrive(0.7, 0.0);	// ...drive straight at 0.7 speed
    				
    				} else if (robotTimer.get() > 4 && robotTimer.get() < 5.8) {	// ...and the time is between 4 and 5.3 seconds...
    					driveTrain.arcadeDrive(0.0, -0.6);	// ...turn
    				}
    			break;
    		}
    	} else {	// If our switch is not to the left (to the right)...
    		switch (robotLocation) {
    			case 0:	// Right Switch ...and we do not know where we are...
    				driveTrain.arcadeDrive(0.0, 0.0);	// ... do not drive
    			break;
    			
				case 1:	// Right Switch ...and we are to the left...
					if (robotTimer.get() > 1.3 && robotTimer.get() < 4) {	// ...and the time is between 0.3 and 1 second(s)...
    					driveTrain.arcadeDrive(0.7, 0.0);	// ...drive straight at 0.7 speed
    					
    				} else if (robotTimer.get() > 4 && robotTimer.get() < 5.8) {	// ...and the time is between 4 and 5.3 seconds...
    					driveTrain.arcadeDrive(0.0, 0.6);
    				} else {
    					driveTrain.arcadeDrive(0.0, 0.0);	// ...stop
    				}
				break;
				case 2:	// Right Switch ...and we are in the middle...
					if (robotTimer.get() > 0.5 && robotTimer.get() < 1.2) {	// ...and the time is between 0.3 and 1 second(s)...
    					driveTrain.arcadeDrive(0.6, 0.0);	// ...drive straight at 0.7 speed
    					
    				} else if (robotTimer.get() > 1.2 && robotTimer.get() < 2.4) {	// ...and the time is between 1 and 1.4 seconds...
    					driveTrain.arcadeDrive(0.0, 0.5);	// ...correct the drift
    					
    				} else if (robotTimer.get() > 2.4 && robotTimer.get() < 4.0) {	// ...and the time is between 1.4 and 4 seconds...
    					driveTrain.arcadeDrive(0.6, 0.0);	// ...drive straight at 0.7 speed
    					
    				} else if (robotTimer.get() > 4.0 && robotTimer.get() < 5.4) {	// ...and the time is between 4 and 5.3 seconds...
    					driveTrain.arcadeDrive(0.0, -0.5);	// ... correct the drift
    					
    				} else if (robotTimer.get() > 5.4 && robotTimer.get() < 6.8) {	// ... and the time is between 5.3 and 5.4 seconds...
    					driveTrain.arcadeDrive(0.7, 0.0);
    					
    				} else if (robotTimer.get() > 6.8 && robotTimer.get() < 7.0) {
    					gripperSolenoid.set(DoubleSolenoid.Value.kForward);	// ...drop the cube
    					
    				} else {	// ...and is none of the above (more than 5.4 seconds)...
    					driveTrain.arcadeDrive(0.0, 0.0);	// ...stop
    				}
				break;
				
				case 3:	// Right Switch  ...and we are to the right...
					if (robotTimer.get() > 1.3 && robotTimer.get() < 4.0) {	// ...and the time is between 0.3 and 1 second(s)...
    					driveTrain.arcadeDrive(0.7, 0.0);	// ...drive straight at 0.7 speed
    				
					} else if (robotTimer.get() > 4 && robotTimer.get() < 4.9) {	// ...and the time is between 4 and 5.3 seconds...
    					driveTrain.arcadeDrive(0.0, -0.6);
    					
    				} else if (robotTimer.get() > 5 && robotTimer.get() < 6) {
    					driveTrain.arcadeDrive(0.6, 0.0);
    					
    				} else if (robotTimer.get() > 6 && robotTimer.get() < 6.1) {	// ... and the time is between 5.3 and 5.4 seconds...
    						gripperSolenoid.set(DoubleSolenoid.Value.kForward);	// ...drop the cube
    						
    				} else {	// ...and is none of the above (more than 5.4 seconds)...
    					driveTrain.arcadeDrive(0.0, 0.0);	// ...stop
    				}
				break;
    		}
    	}
    }

	@Override
	public void teleopPeriodic() {	// Periodic teleop code (run every (10ms?) while teleop is active)
		speed = driveStick.getRawAxis(3) + 1.1;	// Get value of joystick throttle
		driveY = -driveStick.getY() / speed;	// Get value of y axis (f/b), negate (it's negative for some reason) and apply the speed multiplier
		driveX = driveStick.getZ() / 1.5;		// Get the value of z axis (rot), apply constant rotation multiplier

	 	driveTrain.arcadeDrive(driveY, driveX);	// Drive the robot

	 	// Get the values of the buttons
	 	button1Pressed = driveStick.getRawButton(1);
	 	button2Pressed = driveStick.getRawButton(2);
		button3Pressed = driveStick.getRawButton(3);
		button4Pressed = driveStick.getRawButton(4);
		button5Pressed = driveStick.getRawButton(5);
		button6Pressed = driveStick.getRawButton(6);
		button12Pressed = driveStick.getRawButton(12);

		if (armGamepad.getPOV(0) == 180) {button3Pressed = true;}
	 	if (armGamepad.getPOV(0) == 0) {button5Pressed = true;}
	 	if (armGamepad.getRawButton(2) == true) {button4Pressed = true;}
	 	if (armGamepad.getRawButton(4) == true) {button6Pressed = true;}
	 	if (armGamepad.getRawButton(8) == true) {button12Pressed = true;}
	 	if(armGamepad.getRawButton(6)==true) {button1Pressed =true;}
	 	if(armGamepad.getRawButton(5)== true) {button2Pressed = true;}

		if (button12Pressed) {	// If button 12 is pressed...
			armSolenoid.set(true);	// ...raise the arm
		} else {	// If button 12 is not pressed...
			armSolenoid.set(false);	// ...do not raise the arm
		}

		if (button1Pressed && !button2Pressed) {	// If button 1 is pressed...
			gripperL.set(ControlMode.PercentOutput, 1.0);	// ... run the spinny bois forward
			gripperR.set(ControlMode.PercentOutput, -1.0);
		} else if (!button1Pressed && button2Pressed) {	// If button 2 is pressed...
			gripperL.set(ControlMode.PercentOutput, -1.0);	// ... run the spinny bois backward
			gripperR.set(ControlMode.PercentOutput, 1.0);
		} else {	// If nothing or everything is happening...
			gripperL.set(ControlMode.PercentOutput, -.1);	// ... slowly run the spinny bois in to keep the cube in our gripper
			gripperR.set(ControlMode.PercentOutput, -.1);
		}

		if (button5Pressed && !button3Pressed) {	// If button 5 is pressed...
				verticalArm.set(ControlMode.PercentOutput, 0.9);	// ...move the arm up

		} else if (!button5Pressed && button3Pressed) {	// If button 3 is pressed...
				verticalArm.set(ControlMode.PercentOutput, -0.9);	// ...move the arm down

		} else {	// If nothing is pressed...
			verticalArm.set(ControlMode.PercentOutput, 0.0);	// ...stop the arm
		}

		if (button6Pressed && !button4Pressed) {	// If button 6 but not button 4 is pressed...
			gripperSolenoid.set(DoubleSolenoid.Value.kForward);	// ...open the gripper
		} else if (button4Pressed && !button6Pressed) {	// If button 4 but not button 6 is pressed...
			gripperSolenoid.set(DoubleSolenoid.Value.kReverse);	// ...close the gripper
		} else {	// If both or neither button is pressed...
			gripperSolenoid.set(DoubleSolenoid.Value.kOff);	// ...stop moving the gripper
		}
	}
}