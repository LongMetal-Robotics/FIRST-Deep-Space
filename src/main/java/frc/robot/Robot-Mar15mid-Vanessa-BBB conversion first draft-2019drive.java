/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

//Import needed classes
import com.revrobotics.CANSparkMax;             //import class (spark max) from rev robotics
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.lang.model.util.ElementScanner6;

//import org.graalvm.compiler.core.phases.LowTier;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.AnalogInput;  // Import WPILib classes
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    private final int IMG_WIDTH = 320;
    private final int IMG_HEIGHT = 240;
    // Create objects
    private DifferentialDrive driveTrain;   // Drive train
    // Talons
    private PWMVictorSPX topClaw;   // running the top belt on the claw
    private PWMVictorSPX tallDrive; // drive the climb mini drive train
    // Controllers
   	private Joystick driveStickLeft;
   	private Joystick driveStickRight;   // Joystick
	private Joystick armGamepad;    // Gamepad
	private Joystick BBB; 		//Big Button Board (BBB)
	public SendableChooser<String> controls = new SendableChooser<>(); //SendableChooser for drive type
    public String controlType;
    public SendableChooser<String> matchPhase = new SendableChooser<>();  //SendableChooser for button assignment
    public String buttonAssignment;
    // Solenoids
    private DoubleSolenoid climbBack;   //(Double)Solenoids for lifting the robot at the end of the match
	private DoubleSolenoid climbFront;
	private Solenoid climbBackToggle;
    private Solenoid climbFrontToggle;
    private Solenoid clawOpen;
    private Solenoid clawMode;
    //elevator motor
    private CANSparkMax m_elevator;
	private AnalogInput ultrasonicSensor;
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
    double speed = 0.5;     // Drive speed multiplier, initial 0.5
    double driveY = 0.0;    // Drive y (f/b), initial 0
    double driveX = 0.0;    // Drive y (rot), initial 0
    int robotLocation = 0;  // Robot location, failsafe 0 (go nowhere)
    int target = 240;
	private int[] positions = {-1, 30, 60, 120, 240};	// Positions for arm
    int currentPosition = 0;
    public boolean isClosed = false; //true is closed, false is open
    public boolean openMode = false; //true is hatch, false is cargo
    // private NetworkTableEntry sizeEntry;
    // private NetworkTableEntry xEntry;
    // private NetworkTableEntry yEntry;
    // private NetworkTable table;
    private Timer timer;
    public Timer elevatorTimer;
	boolean timerCheck=true;
	public Timer climbTimer;
	boolean climberCheck=false;
	public Timer autoTimer;

	//encoder
	public CANEncoder e_elevator;
	public double elevPos;
	public double tallH;
	public double midH;
	public double lowH;
	public double floorH;
	public boolean tallButton;
	public boolean midButton;
	public boolean lowButton;
	public boolean floorButton;
	

    @Override
    public void robotInit() {   // Run when robot is turned on. Initialize m.controllers, etc.
        // Initialize drive train
        CANSparkMax m_rearLeft = new CANSparkMax(1, MotorType.kBrushless); //left front
        CANSparkMax m_frontLeft = new CANSparkMax(2, MotorType.kBrushless); //left back
        SpeedControllerGroup leftmotors = new SpeedControllerGroup(m_rearLeft, m_frontLeft);
        CANSparkMax m_rearRight = new CANSparkMax(3, MotorType.kBrushless); //right front
        CANSparkMax m_frontRight = new CANSparkMax(4, MotorType.kBrushless); //right back
        SpeedControllerGroup rightmotors = new SpeedControllerGroup(m_rearRight, m_frontRight);
        driveTrain = new DifferentialDrive(leftmotors, rightmotors);
        m_elevator = new CANSparkMax(5, MotorType.kBrushless); //for elevator
		//climb toggle comments
		/*
		climbBack = new DoubleSolenoid(0, 1); //parameters are in and out channels
		climbFront = new DoubleSolenoid(2, 3);
		*/
		///*
		climbBackToggle = new Solenoid(0);
		climbFrontToggle = new Solenoid(2);
		//*/
        clawOpen = new Solenoid(4);
        clawMode = new Solenoid(5);
        ultrasonicSensor = new AnalogInput(0);
        // Initialize arm
        topClaw = new PWMVictorSPX(0);  // Initialize top claw belt motor
        tallDrive = new PWMVictorSPX(1); // Initialize tall drive wheel motor
        // Initialize controllers
        driveStickLeft = new Joystick(0);   // Initialize left joystick
        driveStickRight = new Joystick(1); //Initialize right joystick
		armGamepad = new Joystick(2);   // Initialize gamepad
		BBB = new Joystick(2);	//gamepad and BBB do the same things, chose which one you want to use
		//encoder
		e_elevator = new CANEncoder(m_elevator);
		e_elevator.setPositionConversionFactor(1/(2*Math.PI)); // now, the encoder will read how many inches we are from our defined 0.
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
		
		//controls = new SendableChooser<Integer>();
		//this section sets up all the sendable chooser bits...
		controls.setDefaultOption("Joysticks", "Tank");
        controls.addOption("One joystick", "Arcade");
        controls.addOption("Controller", "Contr");
        controls.addOption("Curvature", "Cheesy");
        controlType = controls.getSelected();
        System.out.println(controlType);
        SmartDashboard.putString("Controls selected", controlType);
        SmartDashboard.putData("Teleop controls", controls);
        matchPhase.setDefaultOption("Teleoperated", "TeleOp");
        matchPhase.addOption("Climb time", "Climb");
        matchPhase.addOption("Already climbed", "PostClimb");
        buttonAssignment = matchPhase.getSelected();
        System.out.println(buttonAssignment);
        SmartDashboard.putString("Button uses", buttonAssignment);
		SmartDashboard.putData("Match Phase", matchPhase);
		
		// this section completes webcam setup
        UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
		camera1.setResolution(IMG_WIDTH, IMG_HEIGHT);
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
		camera2.setResolution(IMG_WIDTH, IMG_HEIGHT);
		
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // table = inst.getTable("GRIP").getSubTable("myBlobsReport");
		timer = new Timer();
		elevatorTimer = new Timer();
		climbTimer = new Timer();
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

	private void armGoToPosition() {
		if (currentPosition != 0 || target != -1) {
			target = positions[currentPosition];
			m_elevator.set(speedToTarget(target, getUltrasonicDistance(ultrasonicSensor)));
		}
	}

    @Override
    public void autonomousInit() {
        timer.reset();
        timer.start();
	}
	
    @Override
    public void autonomousPeriodic() {
        /*sizeEntry = table.getEntry("size");
        xEntry = table.getEntry("x");
        yEntry = table.getEntry("y");
        double size = sizeEntry.getDoubleArray(new double[1])[0];
        centerX = xEntry.getDoubleArray(new double[1])[0];
        double y = yEntry.getDoubleArray(new double[1])[0];
        System.out.println("Blob 0 size: " + size + " (" + centerX + ", " + y + ")");
        driveTrain.arcadeDrive(0.1, ((IMG_WIDTH / 2) - centerX) * 0.01);*/
        System.out.println("Distance: " + getUltrasonicDistance(ultrasonicSensor) + " cm");
        System.out.println("Target: " + target + " cm");
        System.out.println("Distance to target: " + (target - getUltrasonicDistance(ultrasonicSensor)) + " cm");
		System.out.println("Motor Speed: " + (speedToTarget(target, getUltrasonicDistance(ultrasonicSensor)) * 100) + "%");
		System.out.println();
		//teleopPeriodic();
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
        if (armGamepad.getPOV(0) == 180 || BBB.getRawButton(8)) {downArrowPressed = true;} else {downArrowPressed = false;} //elevator down (down arrow)
		if (armGamepad.getPOV(0) == 0 || BBB.getRawButton(9)) {upArrowPressed = true;} else {upArrowPressed = false;} //elevator up (up arrow)
		rightArrowPressed_old = rightArrowPressed;
		if (armGamepad.getPOV(0) == 90) {rightArrowPressed = true;} else {rightArrowPressed = false;} // elevator next target up
		leftArrowPressed_old = leftArrowPressed;
		if (armGamepad.getPOV(0) == 270) {leftArrowPressed = true;} else {leftArrowPressed = false;} // elevator next target down
		buttonAPrev=buttonAPressed;
		if (armGamepad.getRawButton(1) == true || BBB.getRawButton(2) == true) {buttonAPressed = true;} else {buttonAPressed=false;}//open/close claw (A)
		if (armGamepad.getRawButton(2) == true) {buttonBPressed = true;} else {buttonBPressed=false;}//(B)
		buttonXPrev=buttonXPressed;
		if (armGamepad.getRawButton(3) == true || BBB.getRawButton(5) == true) {buttonXPressed = true;} else {buttonXPressed=false;}//Switch opening mode (X)
		buttonYPressed_old = buttonYPressed;
		if (armGamepad.getRawButton(4)==true) {buttonYPressed =true;} else {buttonYPressed=false;}//Both Climb Pistons down (Y)
		if (armGamepad.getRawButton(5)== true) {L1Pressed = true;} else {L1Pressed=false;}//back piston up on elevator (L1)
		if (armGamepad.getRawButton(6)== true) {R1Pressed = true;} else {R1Pressed=false;}//front piston up on elevator (L2)
		if (BBB.getRawButton(6) == true) {tallButton = true;}
		if (BBB.getRawButton(4) == true) {midButton = true;}
		if (BBB.getRawButton(3) == true) {lowButton = true;}
		if (BBB.getRawButton(1) == true) {floorButton = true;}
		
		double LTValue = armGamepad.getRawAxis(2); //left trigger       drive claw back
		double RTValue = armGamepad.getRawAxis(3); //right trigger      drive claw forward
		if(armGamepad.getRawAxis(2) != 0)
		{
			LTValue = armGamepad.getRawAxis(2);
		}
		else if (BBB.getRawButton(7) == true)
		{
			LTValue = 1;
		}
		else if(armGamepad.getRawAxis(3) != 0)
		{
			RTValue = armGamepad.getRawAxis(3);
		}
		else if (BBB.getRawButton(10) == true)
		{
			RTValue = -1;
		}
		else
		{
			RTValue = 0.2;
		}

		//activates the sendable choosers
		SmartDashboard.updateValues();
		controlType=controls.getSelected();
		buttonAssignment = matchPhase.getSelected();

		if (buttonAssignment.equals("TeleOp")) {
			if (controlType.equals("Tank")) {
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
			} else if (controlType.equals("Cheesy")) {
				driveY = driveStickLeft.getY()* ((driveStickLeft.getThrottle() - 1.05) / 2);
				driveX = driveStickRight.getZ() * ((driveStickRight.getThrottle() - 1.05) / -2);
				driveTrain.curvatureDrive(driveY, driveX, true);
			} else /* This is Controller controlType because no others are selected */ {
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
			}
			
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
				  m_elevator.set(-0.4);
			}
			if(!downArrowPressed && !upArrowPressed) {
                timerCheck=true;
                m_elevator.set(0.0);
                //reset the timercheck if the buttons were let go of
			}
			if (leftArrowPressed && !leftArrowPressed_old) {
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
			}
			
			if (RTValue > 0 && LTValue == 0) {
				topClaw.set(RTValue);
			} else if (LTValue > 0 && RTValue == 0) {
				topClaw.set(-LTValue);
			} else {
				topClaw.set(0.0);
			}

			//encoder stuff
			
			if(tallButton)
			{
				if(elevPos < tallH)
				{
					m_elevator.set(0.4);
				}
				else if(elevPos == tallH)
				{
					tallButton = false;
					m_elevator.set(0);
				}
			}
			if(midButton)
			{
				if(elevPos < midH)
				{
					m_elevator.set(0.4);
				}
				else if(elevPos > midH)
				{
					m_elevator.set(-0.4);
				}
				else if(elevPos == midH)
				{
					midButton = false;
					m_elevator.set(0);
				}
			}
			if(lowButton)
			{
				if(elevPos < lowH)
				{
					m_elevator.set(0.4);
				}
				else if(elevPos > lowH)
				{
					m_elevator.set(-0.4);
				}
				else if(elevPos == lowH)
				{
					midButton = false;
					m_elevator.set(0);
				}
			}
			if(floorButton)
			{
				if(elevPos > lowH)
				{
					m_elevator.set(-0.4);
				}
				else if(elevPos == lowH)
				{
					midButton = false;
					m_elevator.set(0);
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

















		} else if (buttonAssignment.equals("Climb")) {
			if(upArrowPressed) { // Elevator
				if(timerCheck){ //reset the timer if the timercheck was reset
					elevatorTimer.reset();
					timerCheck=false;
				}
				if (elevatorTimer.get() <= 0.25 || (!buttonBPressed))
						m_elevator.set(0.4);
				else if (buttonBPressed)
						m_elevator.set(0.8);
				else {
					m_elevator.set(0.4);
				}
			}
      		if(downArrowPressed) {
				  m_elevator.set(-0.4);
			}
			if(!downArrowPressed && !upArrowPressed) {
				timerCheck=true;
                m_elevator.set(0.01);
                //reset the timercheck if the buttons were let go of
			}
			if(buttonYPressed && !buttonYPressed_old) { //button for climb pistons down with offset
				climbTimer.reset();
				climberCheck = true;
				//climb toggle comments
				//climbFront.set(DoubleSolenoid.Value.kReverse);
				climbFrontToggle.set(true);
			}  //put front climb pistons extended
			if (climbTimer.get() >= 5 && climberCheck) {
				//climbBack.set(DoubleSolenoid.Value.kReverse);\
				climbBackToggle.set(true);
				climberCheck = false;
			}
			if(buttonBPressed) { // put both climb pistons down at the same time
				//climbFront.set(DoubleSolenoid.Value.kReverse);
				//climbBack.set(DoubleSolenoid.Value.kReverse);
				climbFrontToggle.set(true);
				climbBackToggle.set(true);
			}
			if(R1Pressed) { //button for front climb up
				//climbFront.set(DoubleSolenoid.Value.kForward);  //put both climb soleniods extended
				climbFrontToggle.set(false);
			}
			if(L1Pressed) { //button for back climb up
				//climbBack.set(DoubleSolenoid.Value.kForward);   //put both climb soleniods extended
				climbBackToggle.set(false);
			}
			
			
			LTValue = armGamepad.getRawAxis(2); //left trigger       drive back
			RTValue = armGamepad.getRawAxis(3); //right trigger      drive forwards
			if (driveStickRight.getY() > 0) {
				tallDrive.set(driveStickRight.getY());
				driveTrain.arcadeDrive(driveStickRight.getY() * 0.25, 0);
			} else if (driveStickRight.getY() < 0) {
				tallDrive.set(driveStickRight.getY());
				driveTrain.arcadeDrive(driveStickRight.getY() * 0.25, 0);
			} else {
				tallDrive.set(0.0);
				driveTrain.arcadeDrive(0, 0);
			}
		} else /* This is PostClimb buttonAssignment because no others are selected */ {
			driveTrain.tankDrive(driveStickLeft.getY() * -0.25, driveStickRight.getY() * -0.25);
		}


	}
	// armGoToPosition();
	/*
		Potentially vision-
		Follow single stripe on the ground
		Align with double stripes on rockets & cargo ship
	*/
}