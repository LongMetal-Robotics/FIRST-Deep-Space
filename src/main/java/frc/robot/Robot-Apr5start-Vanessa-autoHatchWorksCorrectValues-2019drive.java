/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/* LongMetal (c) 2019 FIRST. MIT Commons Licence.                       */
/* Code to start Hartford - Friday, 5th April, 2019. 7:26pm            */
/* Please see Vanessa, Andrew C-P, Sam O, Ranen, or Mr Pearce for edits */
/*----------------------------------------------------------------------*/

package frc.robot;

//Import needed classes
	// Import WPILib classes
import edu.wpi.first.wpilibj.AnalogInput;  
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;

	//import class (spark max) from rev robotics
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;


public class Robot extends TimedRobot {
private static final int _120 = 120;
	// Camera Lag has been experienced, building options for high and low resolution.
private final int IMG_WIDTH_LO = 160;
private final int IMG_HEIGHT_LO = 120;
private final int IMG_WIDTH_HI = 320;
private final int IMG_HEIGHT_HI = 240;
public double autoModifier = 1;

// Create objects in use for the robot.. 
	// Drive train
private DifferentialDrive driveTrain;   
   	// Talons
private PWMVictorSPX topClaw;   // running the top belt on the claw
private PWMVictorSPX tallDrive; // drive the climb mini drive train
  	// Controllers
private Joystick driveStickLeft;
private Joystick driveStickRight;   // Joystick
private Joystick armGamepad;    // Gamepad
private Joystick BBB; 		//Big Button Board (BBB)
  	// Sendable Choosers
public SendableChooser<String> controls = new SendableChooser<>(); //SendableChooser for drive type
public String controlType;
public SendableChooser<String> matchPhase = new SendableChooser<>();  //SendableChooser for button assignment
public String buttonAssignment;
public SendableChooser<String> robotChooser = new SendableChooser<>();  //SendableChooser for button assignment
public String robotSelection;
public SendableChooser<String> tallDriveController = new SendableChooser<>(); //SendableChooser for tall driving controls
public String tallDriveMode;
   	// Solenoids
private DoubleSolenoid climbBack;   //(Double)Solenoids for lifting the robot at the end of the match
private DoubleSolenoid climbFront;
private Solenoid climbBackToggle;
private Solenoid climbFrontToggle;
private Solenoid clawOpen;
private Solenoid clawMode;
   	// Elevator Motor
private CANSparkMax m_elevator;
		// Elevator Encoder
public CANEncoder e_elevator;
		// Drive Encoder
public CANEncoder e_drive;
		// Ultrasonic Sensors
private AnalogInput ultrasonicSensor;
private Ultrasonic leftUltrasonic;
private Ultrasonic rightUltrasonic;
   	// Create variables to store joystick and limit switches values
boolean downArrowPressed = false;
boolean upArrowPressed = false;
boolean rightArrowPressed_old = false;
boolean rightArrowPressed = false;
boolean leftArrowPressed_old = false;
boolean leftArrowPressed = false;
boolean buttonAPressed = false;
boolean buttonAPrev = false;
boolean buttonBPressed = false;
boolean buttonXPressed = false;
boolean buttonXPrev = false;
boolean buttonYPressed_old = false;
boolean buttonYPressed = false;
boolean L1Pressed = false;
boolean R1Pressed = false;
boolean startButtonPressed = false;
boolean selectButtonPressed = false;
    // Motor Speeds Config
double speed = 0.5;     // Drive speed multiplier, initial 0.5
double driveX = 0.0;    // Drive y (f/b), initial 0
double driveZ = 0.0;    // Drive y (rot), initial 0
	// Maybe Legacy 2018 "robot location code"
int robotLocation = 0;  // Robot location, failsafe 0 (go nowhere)
	// Arm Height Code
int target = 240;
// private int[] positions = {-1, 30, 60, 120, 240};	// Positions for arm
int currentPosition = 0;
	// Front Claw Management
public boolean isClosed = false; //true is closed, false is open
public boolean openMode = false; //true is hatch, false is cargo
    
	// private NetworkTableEntry sizeEntry;
	// private NetworkTableEntry xEntry;
	// private NetworkTableEntry yEntry;
	// private NetworkTable table;
	
	// timers
private Timer timer;
public Timer elevatorTimer;
boolean timerCheck=true;
public Timer climbTimer;
boolean climberCheck=false;
public Timer autoTimer;
	
	// More Arm Height Code ?
public double elevPos;
public double tallH;
public double midH;
public double lowH;
public double floorH;
public boolean tallButton;
public boolean midButton;
public boolean lowButton;
public boolean floorButton;

boolean phase00;
boolean phase10;
boolean phase11;
boolean phase12;
	

    @Override
    public void robotInit() {   // Run when robot is turned on. Initialize m.controllers, etc.
        	// Initialize drive train
		CANSparkMax m_rearLeft = new CANSparkMax(1, MotorType.kBrushless); //left front
		m_rearLeft.restoreFactoryDefaults();
		m_rearLeft.setIdleMode(IdleMode.kCoast);
		m_rearLeft.setClosedLoopRampRate(135);
		CANSparkMax m_frontLeft = new CANSparkMax(2, MotorType.kBrushless); //left back
		m_frontLeft.restoreFactoryDefaults();
		m_frontLeft.setIdleMode(IdleMode.kCoast);
		m_frontLeft.setClosedLoopRampRate(135);
        SpeedControllerGroup leftmotors = new SpeedControllerGroup(m_rearLeft, m_frontLeft);
		CANSparkMax m_rearRight = new CANSparkMax(3, MotorType.kBrushless); //right front
		m_rearRight.restoreFactoryDefaults();
		m_rearRight.setIdleMode(IdleMode.kCoast);
		m_rearRight.setClosedLoopRampRate(135);
		CANSparkMax m_frontRight = new CANSparkMax(4, MotorType.kBrushless); //right back
		m_frontRight.restoreFactoryDefaults();
		m_frontRight.setIdleMode(IdleMode.kCoast);
		m_frontRight.setClosedLoopRampRate(135);
        SpeedControllerGroup rightmotors = new SpeedControllerGroup(m_rearRight, m_frontRight);
        driveTrain = new DifferentialDrive(leftmotors, rightmotors);

			// Initialize Elevator Motor
        m_elevator = new CANSparkMax(5, MotorType.kBrushless); 
		

			// Initialize ALL Potential Solenoids for Major Tom & LEO.
				// Major Tom uses Double Solenoids for Climb,
		climbBack = new DoubleSolenoid(0, 1); //parameters are in and out channels
		climbFront = new DoubleSolenoid(2, 3);
				// LEO uses 4-way Solenoids for Climb, and need to be toggled open
		climbBackToggle = new Solenoid(6);
		climbFrontToggle = new Solenoid(7);
				// Both Robots use positions 4 & 5 on the PCM for claw mode and opening.
    	clawOpen = new Solenoid(4);
    	clawMode = new Solenoid(5);

			// Initialize Ultrasonic Sensors and start sending data
		ultrasonicSensor = new AnalogInput(0);

		// Change Sam O 3/24/19 8:42am Add code for sendable ultrasonic sensors for lining up
		leftUltrasonic = new Ultrasonic(0, 1);
		rightUltrasonic = new Ultrasonic(2, 3);
		leftUltrasonic.setAutomaticMode(true);
		rightUltrasonic.setAutomaticMode(true);

        	// Initialize top claw belt motor
        topClaw = new PWMVictorSPX(0);  
			// Initialize tall drive wheel motor
        tallDrive = new PWMVictorSPX(1); 
        
			// Initialize controllers
        driveStickLeft = new Joystick(0);   // Initialize left joystick
        driveStickRight = new Joystick(1); //Initialize right joystick
		armGamepad = new Joystick(2);   // Initialize gamepad
		BBB = new Joystick(2);	//gamepad and BBB do the same things, chose which one you want to use
		
		e_drive = new CANEncoder(m_frontLeft);
		e_drive.setPositionConversionFactor(1 / (2 * Math.PI * 3)); // 3 inch radius for drive wheel
			// Initialize Encoder and Settings
		e_elevator = new CANEncoder(m_elevator);
		e_elevator.setPositionConversionFactor(1/(2*Math.PI*1)/*2/(2*Math.PI)*/); // now, the encoder will read how many inches we are from our defined 0.
		elevPos = 0;
		tallButton = false;
		midButton = false;
		lowButton = false;
		floorButton = false;
		tallH = 75 - 3; //all in inches
		midH = 47 - 3; //inches above the lowest our claw can go
		lowH = 19 - 3;
		floorH = 3 - 3;
		//1 rot = 2pi inches

		//Rocket heights - 19 in, 47 in, 75 in--all 25 in by 16.5 in
		//Cargo ship heights - 19 in--25 in by 16.5 in
		//at the lowest, the claw is 3 inches from the ground
		
		phase00 = false;
		phase10 = true;
		phase11 = true;
		phase12 = false;

			//controls = new SendableChooser<Integer>();
			//this section sets up all the sendable chooser bits...
				// Sendable Chooser #1 - Control Method - Default now Cheesy Drive!
		controls.setDefaultOption("Curvature", "Cheesy");
        controls.addOption("Joysticks", "Tank");
        controls.addOption("One joystick", "Arcade");
        controls.addOption("Controller", "Contr");
        controlType = controls.getSelected();
        System.out.println(controlType);
		SmartDashboard.putString("Controls selected", controlType);
		SmartDashboard.putData("Teleop controls", controls);
				// Sendable Chooser #2 - Match Mode - Default should always be "TeleOp"      
        matchPhase.setDefaultOption("Teleoperated", "TeleOp");
		matchPhase.addOption("Level 3 Climb", "L3 Climb");
		matchPhase.addOption(" ", " ");
		matchPhase.addOption("Level 2 Climb", "L2 Climb");
        //matchPhase.addOption("Already climbed", "PostClimb");
        buttonAssignment = matchPhase.getSelected();
        System.out.println(buttonAssignment);
		SmartDashboard.putString("Button uses", buttonAssignment);
		SmartDashboard.putData("Match Phase", matchPhase);
				// Sendable Chooser #3 - Which Robot?
		robotChooser.setDefaultOption("Major Tom", "MajorTom");
        robotChooser.addOption("Low Earth Orbit", "LEO");
        robotSelection = robotChooser.getSelected();
        System.out.println(robotSelection);
		SmartDashboard.putString("Robot Selected", robotSelection);
		SmartDashboard.putData("Robot Active", robotChooser);
		/*		// Sendable Chooser #4 - What controls the tall driving?
		tallDriveController.setDefaultOption("Gamepad", "Triggers");
        tallDriveController.addOption("Joystick", "Y-Axis");
        tallDriveMode = tallDriveController.getSelected();
        System.out.println(tallDriveMode);
		SmartDashboard.putString("Control Mode", tallDriveMode);
		SmartDashboard.putData("Tall Drive Modes", tallDriveController);
		*/

		
			// this section completes webcam setup
	UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
			// edit the _LO to _HI to increase resolution.....or vice versa
	camera1.setResolution(IMG_WIDTH_LO, IMG_HEIGHT_LO);
			// trying to enforce FPS limit.. please use this method.
	camera1.setFPS(15);
			// we played through all of the pixel formats and "kYUYV" is the best at conserving bandwidth for our microsoft camera.. subject to change if required.
	camera1.setPixelFormat(PixelFormat.kMJPEG);
	UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
			// edit the _LO to _HI to increase resolution.....or vice versa
	camera2.setResolution(IMG_WIDTH_LO, IMG_HEIGHT_LO);
			// trying to enforce FPS limit.. please use this method.
	camera2.setFPS(15);
			// we played through all of the pixel formats and "kYUYV" is the best at conserving bandwidth for our microsoft camera.. subject to change if required.
	camera2.setPixelFormat(PixelFormat.kYUYV);

		
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // table = inst.getTable("GRIP").getSubTable("myBlobsReport");
		
			// this section completes timer setup
		timer = new Timer();
		elevatorTimer = new Timer();
		climbTimer = new Timer();
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Left Ultrasonic Range", (int)leftUltrasonic.getRangeInches());
		SmartDashboard.putNumber("Right Ultrasonic Range", (int)rightUltrasonic.getRangeInches());
		SmartDashboard.putNumber("Ultrasonic Difference", (int)rightUltrasonic.getRangeInches() - (int)leftUltrasonic.getRangeInches());
	}
	
    private int getUltrasonicDistance(AnalogInput sensor) {
        double rawVoltage = sensor.getVoltage();
        int cm = (int)(rawVoltage * 102.4);
        return cm;
	}
	
    private int speedToTarget(int target, int current) {
        double multiplier = 80 / 15.46;
        int direction = 0;
        if (target > current) {
            direction = 1;
        } else if (target < current) {
            direction = -1;
        } else {
            direction = 0;
        }
        int speed = (int)(multiplier * Math.sqrt(direction * (target - current)));
        return speed;
	}

	/* private void armGoToPosition() {
		if (currentPosition != 0 || target != -1) {
			target = positions[currentPosition];
			m_elevator.set(speedToTarget(target, getUltrasonicDistance(ultrasonicSensor)));
		}
	} */

    @Override
    public void autonomousInit() {
        timer.reset();
        timer.start();
	}
	
    @Override
    public void autonomousPeriodic() {
		autoModifier = .5;
		teleopPeriodic();
        /*sizeEntry = table.getEntry("size");
        xEntry = table.getEntry("x");
        yEntry = table.getEntry("y");
        double size = sizeEntry.getDoubleArray(new double[1])[0];
        centerX = xEntry.getDoubleArray(new double[1])[0];
        double y = yEntry.getDoubleArray(new double[1])[0];
        System.out.println("Blob 0 size: " + size + " (" + centerX + ", " + y + ")");
        driveTrain.arcadeDrive(0.1, ((IMG_WIDTH / 2) - centerX) * 0.01);*/
        /*System.out.println("Distance: " + getUltrasonicDistance(ultrasonicSensor) + " cm");
        System.out.println("Target: " + target + " cm");
        System.out.println("Distance to target: " + (target - getUltrasonicDistance(ultrasonicSensor)) + " cm");
		System.out.println("Motor Speed: " + (speedToTarget(target, getUltrasonicDistance(ultrasonicSensor)) * 100) + "%");
		System.out.println();*/
	}

	@Override
	public void teleopInit() {
		timer.reset();
		timer.start();
		elevatorTimer.reset();
		elevatorTimer.start();
		climbTimer.reset();
		climbTimer.start();
	}

    @Override
	public void teleopPeriodic() {  // Periodic teleop code (run every (10ms?) while teleop is active)
		//button boi hours
		downArrowPressed = armGamepad.getPOV(0) == 180;
		upArrowPressed = armGamepad.getPOV(0) == 0;
		rightArrowPressed_old = rightArrowPressed;
		rightArrowPressed = armGamepad.getPOV(0) == 90;
		leftArrowPressed_old = leftArrowPressed;
		leftArrowPressed = armGamepad.getPOV(0) == 270;
		buttonAPrev=buttonAPressed;
		buttonAPressed = armGamepad.getRawButton(1); //open/close claw (A)
		buttonBPressed = armGamepad.getRawButton(2); //(B)
		buttonXPrev = buttonXPressed;
		if (armGamepad.getRawButton(3)) {buttonXPressed = true;} else {buttonXPressed=false;}//Switch opening mode (X)
		buttonYPressed_old = buttonYPressed;
		if (armGamepad.getRawButton(4)) {buttonYPressed = true;} else {buttonYPressed=false;}//Both Climb Pistons down (Y)
		if (armGamepad.getRawButton(5)) {L1Pressed = true;} else {L1Pressed=false;}//back piston up on elevator (L1)
		if (armGamepad.getRawButton(6)) {R1Pressed = true;} else {R1Pressed=false;}//front piston up on elevator (L2)
		if (armGamepad.getRawButton(7)) {selectButtonPressed = true;} else {selectButtonPressed = false;}
		if (armGamepad.getRawButton(8)) {startButtonPressed = true;} //else {startButtonPressed = false;} //start button, so the auto code
		double LTValue = armGamepad.getRawAxis(2); //left trigger       drive claw back
		double RTValue = armGamepad.getRawAxis(3); //right trigger      drive claw forward
		

		//activates the sendable choosers
		SmartDashboard.updateValues();
		controlType = controls.getSelected();
		buttonAssignment = matchPhase.getSelected();
		robotSelection = robotChooser.getSelected();
		//tallDriveMode = tallDriveController.getSelected();
		
		SmartDashboard.putNumber("Encoder, in", e_elevator.getPosition());
		SmartDashboard.putString("Controls selected", controlType);
		SmartDashboard.putString("Button uses", buttonAssignment);
		SmartDashboard.putString("Robot Selected", robotSelection);
		//SmartDashboard.putString("Control Mode", tallDriveMode);

		if (buttonAssignment.equals("TeleOp")) {
			tallDrive.set(0);
			/*if (controlType.equals("Tank")) {
				if (driveStickLeft.getRawButton(1) && !driveStickRight.getRawButton(1)) {
					double y = driveStickLeft.getY() * driveStickLeft.getThrottle();
					driveTrain.arcadeDrive(-1 * y, 0);
				} else if (driveStickRight.getRawButton(1) && !driveStickLeft.getRawButton(1)) {
					double y = driveStickRight.getY() * driveStickRight.getThrottle();
					driveTrain.arcadeDrive(-1 * y, 0);
				} else if ((driveStickLeft.getRawButton(1) && driveStickRight.getRawButton(1)) && Math.abs(driveStickLeft.getY()) < Math.abs(driveStickRight.getY())) {
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
			} else if (controlType.equals("Arcade")) {
				speed = driveStickLeft.getRawAxis(3) + 1.1; // Get value of joystick throttle
				driveY = -driveStickLeft.getY() / speed;
				driveX = driveStickLeft.getZ() / 1.5;
				driveTrain.arcadeDrive(driveY, driveX);
			} else if (controlType.equals("Cheesy")) {*/
				double modifier = ((0.7 * driveStickLeft.getThrottle() - 1.05) / 2);
				if (robotSelection.equals("MajorTom")) {
					double left = leftUltrasonic.getRangeInches();
					double right = rightUltrasonic.getRangeInches();
					if (left <= 15 && right <= 15) {
						modifier *= 0.5;
					}
				}
				driveX = autoModifier * driveStickLeft.getY() * modifier;
				driveZ = autoModifier * driveStickRight.getZ() * (driveStickRight.getThrottle() - 1) * -0.5;
				driveTrain.curvatureDrive(driveX, driveZ, true);
			/*} else *//* This is Controller controlType because no others are selected *//* {
				if (armGamepad.getRawButton(5) && !armGamepad.getRawButton(6)) {
					double y = -1 * armGamepad.getRawAxis(1);
					driveTrain.arcadeDrive(y, 0);
				} else if (!armGamepad.getRawButton(5) && armGamepad.getRawButton(6)) {
					double y = -1 * armGamepad.getRawAxis(5);
					driveTrain.arcadeDrive(y, 0);
				} else if ((armGamepad.getRawButton(5) && armGamepad.getRawButton(6)) && Math.abs(armGamepad.getRawAxis(1)) < Math.abs(armGamepad.getRawAxis(2))) {
					double y = -1 * armGamepad.getRawAxis(1);
					driveTrain.arcadeDrive(y, 0);
				} else if ((armGamepad.getRawButton(5) && armGamepad.getRawButton(6)) && Math.abs(armGamepad.getRawAxis(1)) > Math.abs(armGamepad.getRawAxis(2))) {
					double y = -1 * armGamepad.getRawAxis(5);
					driveTrain.arcadeDrive(y, 0);
				} else {
					double left = -1 * armGamepad.getRawAxis(1);
					double right = -1 * armGamepad.getRawAxis(5);
					driveTrain.tankDrive(left, right);
				}
			}*/
			
			if(buttonAPressed && !buttonAPrev) {
				if(isClosed) {
					clawOpen.set(false);
					isClosed = false;
				} else {
					clawOpen.set(true);
					isClosed = true;
				}
			}
			if(buttonXPressed && !buttonXPrev) {
				if(openMode && !isClosed) {
					clawMode.set(false);
					openMode = false;
				}
				else if(!isClosed) {
					clawMode.set(true);
					openMode = true;
				}
			}
			if(upArrowPressed) { // Elevator
				if(timerCheck){ //reset the timer if the timercheck was reset
					elevatorTimer.reset();
					timerCheck=false;
				}
				if (elevatorTimer.get() <= 0.25 || !buttonBPressed)
						m_elevator.set(0.4);
				else if (buttonBPressed)
						m_elevator.set(0.8);
			}

			//if(!upArrowPressed && !downArrowPressed && elevatorTimer.get() == 0)
			//	m_elevator.set(0.4);
			
      		if(downArrowPressed) {
				if (elevatorTimer.get() <= 0.25 || !buttonBPressed)
					 m_elevator.set(-0.4);
				else if (buttonBPressed)
					m_elevator.set(-0.8);
			}
			if(!downArrowPressed && !upArrowPressed) {
                timerCheck=true;
                m_elevator.set(0.0);
                //reset the timercheck if the buttons were let go of
			}
			/*if (leftArrowPressed && !leftArrowPressed_old) {
				// left arrow ondown
				// next position down
				currentPosition--;
				if (currentPosition < 1) {
					currentPosition = 1;
				}
				// if (currentPosition < 1) {
				// 	currentPosition = positions.length - 1;
				// }
			} else if (rightArrowPressed && !rightArrowPressed_old) {
				// right arrow ondown
				// next position up
				currentPosition++;
				if (currentPosition >= positions.length) {
					currentPosition = positions.length - 1;
				}
			}*/
			
			if (RTValue > 0 && LTValue == 0) {
				topClaw.set(RTValue);
			} else if (LTValue > 0 && RTValue == 0) {
				topClaw.set(-LTValue);
			} else if (RTValue == 0 && LTValue == 0) {
				topClaw.set(0.0);
			}

			//encoder stuff
			
			// if(tallButton)
			// {
			// 	if(elevPos < tallH)
			// 	{
			// 		m_elevator.set(0.4);
			// 	}
			// 	else if(elevPos == tallH)
			// 	{
			// 		tallButton = false;
			// 		m_elevator.set(0);
			// 	}
			// }
			// if(midButton)
			// {
			// 	if(elevPos < midH)
			// 	{
			// 		m_elevator.set(0.4);
			// 	}
			// 	else if(elevPos > midH)
			// 	{
			// 		m_elevator.set(-0.4);
			// 	}
			// 	else if(elevPos == midH)
			// 	{
			// 		midButton = false;
			// 		m_elevator.set(0);
			// 	}
			// }
			// if(lowButton)
			// {
			// 	if(elevPos < lowH)
			// 	{
			// 		m_elevator.set(0.4);
			// 	}
			// 	else if(elevPos > lowH)
			// 	{
			// 		m_elevator.set(-0.4);
			// 	}
			// 	else if(elevPos == lowH)
			// 	{
			// 		midButton = false;
			// 		m_elevator.set(0);
			// 	}
			// }
			// if(floorButton)
			// {
			// 	if(elevPos > lowH)
			// 	{
			// 		m_elevator.set(-0.4);
			// 	}
			// 	else if(elevPos == lowH)
			// 	{
			// 		midButton = false;
			// 		m_elevator.set(0);
			// 	}
			// }

			//Just trying to see if auto heights can/will work with the encoder by setting a specific height then measuring to see if it's the correct height irl
			/*if (e_elevator.getPosition() < 36)
				m_elevator.set(0.4);
			else if (e_elevator.getPosition() > 36)
				m_elevator.set(-0.4);
			else
				m_elevator.set(0);*/
			
			
			if(selectButtonPressed)
			{
				phase00 = false;
				e_elevator.setPosition(0.0);
				e_drive.setPosition(0.0);
				startButtonPressed = false;
			}
			if(startButtonPressed)
			{
				phase00 = true;
			}

			if(phase00) { 
				if(phase11)
				{
					if(e_elevator.getPosition() < 4 /*height untill we are unhooked aaaand to get to the correct height*/ )
					{
						m_elevator.set(0.4);
					}
					else 
					{
						m_elevator.set(0.0);
						phase11 = false;
					}
				}
				if(phase10)
				{
					// 3in (bumper) + 38in (back to hatch on the claw)
					// 27 ft (half of field) - 4ft(hab level 2 platform) - 7ft 4.25in (length of cargo) = back of level 1 hab to front face of cargo 15ft 7.75in
					//so 187.75 in - 41 in = 146.25 length we have to move
					if(e_drive.getPosition() < 146.25)
					{
						driveTrain.arcadeDrive(0.4, 0.0);
					}
					else
					{
						driveTrain.arcadeDrive(0.0, 0.0);
						phase10 = false;
						startButtonPressed = false;
					}
				}
			}
			
			//spooky autolineup code
			//okay so we have two ultrasonic sensors, one on each side of the front of the drive train
			//so lets say we have two variables that hold the distance in inches
			//double distanceLeft;
			//double distanceRight;
			//if(false /*we want to autolineup*/)
			/*{
				autoTimer.reset();	// Set timer to 0
				autoTimer.start();
				if(distanceLeft < distanceRight) //if left is closer
				{
					if(autoTimer.get() < 0.3)
					{
						driveTrain.arcadeDrive(0, 0.6); //x:speed y:rotation
					}
				}
			*/

		} else if (buttonAssignment.equals("L3 Climb")) {
			if(upArrowPressed) { // Elevator
				if(timerCheck){ //reset the timer if the timercheck was reset
					elevatorTimer.reset();
					timerCheck=false;
				}
				if (elevatorTimer.get() <= 0.25 || (!buttonBPressed))
						m_elevator.set(0.4);
				else if (buttonBPressed)
						m_elevator.set(0.95);
				else {
					m_elevator.set(0.4);
				}
			}
      		if(downArrowPressed) { // Elevator
				if(timerCheck){ //reset the timer if the timercheck was reset
					elevatorTimer.reset();
					timerCheck=false;
				}
				if (elevatorTimer.get() <= 0.25 || (!buttonBPressed))
						m_elevator.set(-0.4);
				else if (buttonBPressed)
						m_elevator.set(-0.95);
				else {
					m_elevator.set(-0.4);
				}
			}
			if(!downArrowPressed && !upArrowPressed) {
				timerCheck=true;
                m_elevator.set(0.01);
                //reset the timercheck if the buttons were let go of
			}
			if(buttonYPressed && !buttonYPressed_old) { //button for climb pistons down with timed offset
				climbTimer.reset();
				climberCheck = true;
				// Deploy Front Climb Pistons (extend)
				if (robotSelection.equals("MajorTom"))
					climbFront.set(DoubleSolenoid.Value.kReverse);
				else if (robotSelection.equals("LEO"))
					climbFrontToggle.set(true);
			}  // Now Wait 4 Seconds and Deploy Rear Climb Piston
			if (climbTimer.get() >= 4 && climberCheck) {
				if (robotSelection.equals("MajorTom"))
					climbBack.set(DoubleSolenoid.Value.kReverse);
				else if (robotSelection.equals("LEO"))
					climbBackToggle.set(true);
				climberCheck = false;
			}
			/* if(buttonBPressed) { // put both climb pistons down at the same time
				if (robotSelection.equals("MajorTom")) {
					climbFront.set(DoubleSolenoid.Value.kReverse);
					climbBack.set(DoubleSolenoid.Value.kReverse); }
				else if (robotSelection.equals("LEO")) {
					climbFrontToggle.set(true);
					climbBackToggle.set(true); }
			} */
			if(R1Pressed) { // Retract Front Climb Pistons
				if (robotSelection.equals("MajorTom"))
					climbFront.set(DoubleSolenoid.Value.kForward);
				else if (robotSelection.equals("LEO"))
					climbFrontToggle.set(false);
			}
			if(L1Pressed) { // Retract Rear Climb Piston
				if (robotSelection.equals("MajorTom"))
					climbBack.set(DoubleSolenoid.Value.kForward);
				else if (robotSelection.equals("LEO"))
					climbBackToggle.set(false);
			}
			
			//climb time drive
			
			//LTValue = armGamepad.getRawAxis(2); //left trigger       drive back
			//RTValue = armGamepad.getRawAxis(3); //right trigger      drive forwards
			
			if (RTValue > 0 && LTValue == 0) {	
				driveTrain.arcadeDrive(RTValue * 0.25, 0);
				tallDrive.set(RTValue);
			} else if (LTValue > 0 && RTValue == 0) {
				tallDrive.set(-LTValue);					
				driveTrain.arcadeDrive(-LTValue * 0.25, 0);
			} else if (RTValue == 0 && LTValue == 0) {
				tallDrive.set(0.0);
				driveTrain.arcadeDrive(0, 0);
			}
			
					
		} else if (buttonAssignment.equals("L2 Climb"))
		{
			if(upArrowPressed) { // Elevator
				if(timerCheck){ //reset the timer if the timercheck was reset
					elevatorTimer.reset();
					timerCheck=false;
				}
				if (elevatorTimer.get() <= 0.25 || (!buttonBPressed))
						m_elevator.set(0.4);
				else if (buttonBPressed)
						m_elevator.set(0.95);
				else {
					m_elevator.set(0.4);
				}
			}
      		if(downArrowPressed) { // Elevator
				if(timerCheck){ //reset the timer if the timercheck was reset
					elevatorTimer.reset();
					timerCheck=false;
				}
				if (elevatorTimer.get() <= 0.25 || (!buttonBPressed))
						m_elevator.set(-0.4);
				else if (buttonBPressed)
						m_elevator.set(-0.95);
				else {
					m_elevator.set(-0.4);
				}
			}
			if(!downArrowPressed && !upArrowPressed) {
				timerCheck=true;
                m_elevator.set(0.01);
                //reset the timercheck if the buttons were let go of
			}
			if(buttonYPressed && !buttonYPressed_old) { //button for climb pistons down with timed offset
				climbTimer.reset();
				climberCheck = true;
				// Deploy Front Climb Pistons (extend)
				if (robotSelection.equals("MajorTom"))
					climbFront.set(DoubleSolenoid.Value.kReverse);
				else if (robotSelection.equals("LEO"))
					climbFrontToggle.set(true);
			}  // Now Wait 1.5 Seconds and Deploy Rear Climb Piston
			if (climbTimer.get() >= 1.5 && climberCheck) {
				if (robotSelection.equals("MajorTom"))
					climbBack.set(DoubleSolenoid.Value.kReverse);
				else if (robotSelection.equals("LEO"))
					climbBackToggle.set(true);
				climberCheck = false;
			}
			/* if(buttonBPressed) { // put both climb pistons down at the same time
				if (robotSelection.equals("MajorTom")) {
					climbFront.set(DoubleSolenoid.Value.kReverse);
					climbBack.set(DoubleSolenoid.Value.kReverse); }
				else if (robotSelection.equals("LEO")) {
					climbFrontToggle.set(true);
					climbBackToggle.set(true); }
			} */
			if(R1Pressed) { // Retract Front Climb Pistons
				if (robotSelection.equals("MajorTom"))
					climbFront.set(DoubleSolenoid.Value.kForward);
				else if (robotSelection.equals("LEO"))
					climbFrontToggle.set(false);
			}
			if(L1Pressed) { // Retract Rear Climb Piston
				if (robotSelection.equals("MajorTom"))
					climbBack.set(DoubleSolenoid.Value.kForward);
				else if (robotSelection.equals("LEO"))
					climbBackToggle.set(false);
			}
			
			//climb time drive
			
			//LTValue = armGamepad.getRawAxis(2); //left trigger       drive back
			//RTValue = armGamepad.getRawAxis(3); //right trigger      drive forwards
			
			if (RTValue > 0 && LTValue == 0) {	
				driveTrain.arcadeDrive(RTValue * 0.25, 0);
				tallDrive.set(RTValue);
			} else if (LTValue > 0 && RTValue == 0) {
				tallDrive.set(-LTValue);					
				driveTrain.arcadeDrive(-LTValue * 0.25, 0);
			} else if (RTValue == 0 && LTValue == 0) {
				tallDrive.set(0.0);
				driveTrain.arcadeDrive(0, 0);
			}
			
					
					
		} else /* This is PostClimb buttonAssignment because no others are selected */ {
			// this line commented out by Andrew CP because we have no current use for driving once out of climb time.
			// driveTrain.tankDrive(driveStickLeft.getY() * -0.25, driveStickRight.getY() * -0.25);
		}


	}
	// armGoToPosition();
	/*
		Potentially vision-
		Follow single stripe on the ground
		Align with double stripes on rockets & cargo ship
	*/
}