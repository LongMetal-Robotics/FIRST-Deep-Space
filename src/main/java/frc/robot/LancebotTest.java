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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
	private final int IMG_WIDTH = 320;
	private final int IMG_HEIGHT = 240;

	// Create objects
	private DifferentialDrive driveTrain;	// Drive train
	// Talons
	private TalonSRX verticalArm;	// Vertical motor
	//private TalonSRX gripperL;	// Gripper Motors
	//private TalonSRX gripperR;
	// Controllers
  private Joystick driveStickLeft;
  private Joystick driveStickRight;	// Joystick
	private Joystick armGamepad;	// Gamepad
	// Solenoids
	private DoubleSolenoid gripperSolenoid;	// Gripper solenoid

	private final Object imgLock = new Object();
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
		Spark m_rearLeft = new Spark(0);	// Initialize left Sparks
		Spark m_frontLeft = new Spark(1);

		SpeedControllerGroup m_left = new SpeedControllerGroup(m_rearLeft, m_frontLeft);	// Join left Sparks into a group

		Spark m_frontRight = new Spark(2);	// Initialize right Sparks
		Spark m_rearRight = new Spark(3);
		SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);	// Join right Sparks into a group

		driveTrain = new DifferentialDrive(m_left, m_right);	// Create drive train with Spark groups
		
		gripperSolenoid = new DoubleSolenoid(0, 1);
		// Initialize arm
		verticalArm = new TalonSRX(0);	// Initialize vertical arm motor
		/*gripperL = new TalonSRX(1);	// Initialize gripper Talons
		gripperR = new TalonSRX(2);*/
		
		// Initialize controllers
    driveStickLeft = new Joystick(0);	// Initialize left joystick
    driveStickRight = new Joystick(1); //Initialize right joystick
		armGamepad = new Joystick(2);	// Initialize gamepad

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
		double x = xEntry.getDoubleArray(new double[1])[0];
		double y = yEntry.getDoubleArray(new double[1])[0];

		System.out.println("Blob 0 size: " + size + " (" + x + ", " + y + ")");

		driveTrain.arcadeDrive(0.1, ((IMG_WIDTH / 2) - x) * 0.01);
	}

	@Override
	public void teleopPeriodic() {	// Periodic teleop code (run every (10ms?) while teleop is active)
	
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

		button3Pressed = false;
		button5Pressed = false;
		if (armGamepad.getPOV() == 180) {button3Pressed = true; /*System.out.println("[INFO] POV Down");*/}
		if (armGamepad.getPOV() == 0) {button5Pressed = true; /*System.out.println("[INFO] POV Up");*/}
		if (armGamepad.getRawButton(2) == true) {button4Pressed = true; /*System.out.println("[INFO] Close Gripper");*/} else { button4Pressed = false;}
		if (armGamepad.getRawButton(4) == true) {button6Pressed = true; /*System.out.println("[INFO] Open Gripper");*/} else { button6Pressed = false;}

		if (button5Pressed && !button3Pressed) {	// If button 5 is pressed...
			// System.out.println("[INFO] Moving arm up");
				verticalArm.set(ControlMode.PercentOutput, 1);	// ...move the arm up

		} else if (!button5Pressed && button3Pressed) {	// If button 3 is pressed...
			// System.out.println("[INFO] Moving arm down");
				verticalArm.set(ControlMode.PercentOutput, -0.9);	// ...move the arm down

		} else {	// If nothing is pressed...
			// System.out.println("[INFO] Stopping arm");
			verticalArm.set(ControlMode.PercentOutput, 0.0);	// ...stop the arm
		}

		if (button6Pressed && !button4Pressed) {	// If button 6 but not button 4 is pressed...
			// System.out.println("[INFO] Opening Gripper");
			gripperSolenoid.set(DoubleSolenoid.Value.kForward);	// ...open the gripper
		} else if (button4Pressed && !button6Pressed) {	// If button 4 but not button 6 is pressed...
			// System.out.println("[INFO] Closing Gripper");
			gripperSolenoid.set(DoubleSolenoid.Value.kReverse);	// ...close the gripper
		} else {	// If both or neither button is pressed...
			// System.out.println("[INFO] Disabling gripper");
			gripperSolenoid.set(DoubleSolenoid.Value.kOff);	// ...stop moving the gripper
		}
	}
}