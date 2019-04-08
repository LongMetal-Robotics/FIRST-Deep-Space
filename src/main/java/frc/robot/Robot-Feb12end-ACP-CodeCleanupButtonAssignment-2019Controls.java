/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import javax.lang.model.util.ElementScanner6;

//Import needed classes
import com.ctre.phoenix.motorcontrol.ControlMode;	// Import classes from CTRE
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANDigitalInput; 			//import class (spark max) from rev robotics
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.CameraServer;	// Import WPILib classes
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.Sendable;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
	private final int IMG_WIDTH = 320;
	private final int IMG_HEIGHT = 240;

	// Create objects
	private DifferentialDrive driveTrain;	// Drive train
	// Talons
	private TalonSRX topClaw;	// running the top belt on the claw
	private TalonSRX tallDrive;	// drive the climb mini drive train
	
	// Controllers
  	private Joystick driveStickLeft;
  	private Joystick driveStickRight;	// Joystick
	private Joystick armGamepad;	// Gamepad

 	public SendableChooser<String> controls = new SendableChooser<>(); //SendableChooser for drive type
	public String controlType;

	public SendableChooser<String> matchPhase = new SendableChooser<>();  //SendableChooser for button assignment
	public String buttonAssignment;

	// Solenoids
	private DoubleSolenoid climbBack;	//(Double)Solenoids for lifting the robot at the end of the match
	private DoubleSolenoid climbFront;
	private Solenoid clawLeft;			//(Single)Solenoids for opening and closing the arm
	private Solenoid clawRight;

	private DigitalInput jumpA;		//In case we use jumpers for position again this year
	private DigitalInput jumpB;


	//vision?
	private double centerX = 0.0;



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


	private NetworkTableEntry sizeEntry;
	private NetworkTableEntry xEntry;
	private NetworkTableEntry yEntry;
	private NetworkTable table;

	@Override
	public void robotInit() {	// Run when robot is turned on. Initialize m.controllers, etc.
		// Initialize drive train
		CANSparkMax m_rearLeft = new CANSparkMax(0, MotorType.kBrushless); //left front
		CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless); //left back
		SpeedControllerGroup leftmotors = new SpeedControllerGroup(m_rearLeft, m_frontLeft);


		CANSparkMax m_rearRight = new CANSparkMax(2, MotorType.kBrushless); //right front
		CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushless); //right back
		SpeedControllerGroup rightmotors = new SpeedControllerGroup(m_rearRight, m_frontRight);

		driveTrain = new DifferentialDrive(leftmotors, rightmotors);

		CANSparkMax m_elevator = new CANSparkMax(4, MotorType.kBrushless); //for elevator
		
		climbBack = new DoubleSolenoid(0, 1);
		climbFront = new DoubleSolenoid(0, 1);
		clawLeft = new Solenoid(2);
		clawRight = new Solenoid(2);

		jumpA = new DigitalInput(2);
		jumpB = new DigitalInput(3);


		// Initialize arm
		topClaw = new TalonSRX(0);	// Initialize top claw belt motor
		tallDrive = new TalonSRX(1); // Initialize tall drive wheel motor
		
		
		// Initialize controllers
    driveStickLeft = new Joystick(0);	// Initialize left joystick
    driveStickRight = new Joystick(1); //Initialize right joystick
	armGamepad = new Joystick(2);	// Initialize gamepad

	//controls = new SendableChooser<Integer>();
	controls.setDefaultOption("Joysticks", "Tank");
	controls.addOption("One joystick", "Arcade");
	controls.addOption("Controller", "Contr");
	controlType = controls.getSelected();
	System.out.println(controlType);
	SmartDashboard.putString("Controls selected", controlType);
	SmartDashboard.putData("Teleop controls", controls);

	matchPhase.setDefaultOption("Teleoperated", "Regular Uses");
	matchPhase.addOption("Climb time", "Climbing assignments");
	matchPhase.addOption("Already climbed", "Can't do both");
	buttonAssignment = matchPhase.getSelected();
	System.out.println(buttonAssignment);
	SmartDashboard.putString("Button uses", buttonAssignment);
	SmartDashboard.putData("Match Phase", matchPhase);
			// Find new camera server code!!!  NP 1/22/2019
		 //CameraServer.getInstance().startAutomaticCapture();	// Start camera stream to DS

		 UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		table = inst.getTable("GRIP").getSubTable("myBlobsReport");
	}

	@Override
	public void autonomousPeriodic() {
		sizeEntry = table.getEntry("size");
		xEntry = table.getEntry("x");
		yEntry = table.getEntry("y");

		double size = sizeEntry.getDoubleArray(new double[1])[0];
		centerX = xEntry.getDoubleArray(new double[1])[0];
		double y = yEntry.getDoubleArray(new double[1])[0];

		System.out.println("Blob 0 size: " + size + " (" + centerX + ", " + y + ")");

		driveTrain.arcadeDrive(0.1, ((IMG_WIDTH / 2) - centerX) * 0.01);
	}

	@Override
	public void teleopPeriodic() {	// Periodic teleop code (run every (10ms?) while teleop is active)
		SmartDashboard.updateValues();
		controlType=controls.getSelected();
		System.out.println(controlType);
		if (controlType == "Tank")
		{	
			if (driveStickLeft.getRawButton(1) && !driveStickRight.getRawButton(1)) {
				double y = driveStickLeft.getY() * driveStickLeft.getThrottle();
				driveTrain.arcadeDrive(-1 * y, 0);
			} else if (driveStickRight.getRawButton(1) && !driveStickLeft.getRawButton(1)) {
				double y = driveStickRight.getY() * driveStickRight.getThrottle();
				driveTrain.arcadeDrive(-1 * y, 0);
			} else if ((driveStickLeft.getRawButton(1) && driveStickRight.getRawButton(1))&&Math.abs(driveStickLeft.getY())<Math.abs(driveStickRight.getY())) {
				double y = driveStickLeft.getY() * driveStickLeft.getThrottle();
				driveTrain.arcadeDrive(-1 * y, 0);
			} else if ((driveStickLeft.getRawButton(1) && driveStickRight.getRawButton(1))&&Math.abs(driveStickRight.getY())<Math.abs(driveStickLeft.getY())) {
				double y = driveStickRight.getY() * driveStickRight.getThrottle();
				driveTrain.arcadeDrive(-1 * y, 0);
			} else {
				double left = driveStickLeft.getY() * driveStickLeft.getThrottle();
				double right = driveStickRight.getY() * driveStickRight.getThrottle();
				driveTrain.tankDrive(-1 * left, -1 * right);
			}
		} else if (controlType == "Arcade")
		{
			speed = driveStickLeft.getRawAxis(3) + 1.1;	// Get value of joystick throttle
			driveY = -driveStickLeft.getY() / speed;
			driveX = driveStickLeft.getZ() / 1.5;
			driveTrain.arcadeDrive(driveY, driveX);
		} else
		{
			if (armGamepad.getRawButton(5)&&!armGamepad.getRawButton(6))
   			{
    		  double y = -1 * armGamepad.getRawAxis(1);
    		  driveTrain.arcadeDrive(y, 0);
    		} else if (!armGamepad.getRawButton(5)&&armGamepad.getRawButton(6))
    		{
    		  double y = -1 * armGamepad.getRawAxis(5);
    		  driveTrain.arcadeDrive(y, 0);
    		} else if ((armGamepad.getRawButton(5)&&armGamepad.getRawButton(6))&&Math.abs(armGamepad.getRawAxis(1))<Math.abs(armGamepad.getRawAxis(2)))
    		{
    		  double y = -1 * armGamepad.getRawAxis(1);
    		  driveTrain.arcadeDrive(y, 0);
    		} else if ((armGamepad.getRawButton(5)&&armGamepad.getRawButton(6))&&Math.abs(armGamepad.getRawAxis(1))>Math.abs(armGamepad.getRawAxis(2)))
    		{
    		  double y = -1 * armGamepad.getRawAxis(5);
    		  driveTrain.arcadeDrive(y, 0);
    		} else
    		{
    		  double left = -1 * armGamepad.getRawAxis(1);
    		  double right = -1 * armGamepad.getRawAxis(5);
    		  driveTrain.tankDrive(left, right);
			}
		}
		

		button3Pressed = false;
		button5Pressed = false;
		if (armGamepad.getPOV() == 180) {button3Pressed = true; /*System.out.println("[INFO] POV Down");*/}
		if (armGamepad.getPOV() == 0) {button5Pressed = true; /*System.out.println("[INFO] POV Up");*/}
		if (armGamepad.getRawButton(2) == true) {button4Pressed = true; /*System.out.println("[INFO] Close Gripper");*/} else { button4Pressed = false;}
		if (armGamepad.getRawButton(4) == true) {button6Pressed = true; /*System.out.println("[INFO] Open Gripper");*/} else { button6Pressed = false;}
		
		//button boi hours
		
		//buttons we need

		/* Claw-
			Priority- Ready to use, fully closed
			Potential- Fully open, partially open
		   Climbing-
			Toggle both (only if not already climbed and both are the same)
			Toggle back one (potentially only if front one is shut)
			Toggle front one
		   Elevator-
			Move up
			Move down
		   Potentially vision-
			Follow single stripe on the ground
			Align with double stripes on rockets & cargo ship
		*/
		
		
		}
	}
