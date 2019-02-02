/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot;

import javax.lang.model.util.ElementScanner6;

// Import dependencies
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CANSparkMax leftFront, leftRear, rightFront, rightRear; // Drive Train Speed Controllers (Spark MAX -> NEO)
  private DifferentialDrive driveTrain;

  private CANSparkMax elevator; // Motor for raising/lowering elevator

  private DoubleSolenoid clawOpenCloseSolenoid;

  private DoubleSolenoid stopperPiston, frontClimbSolenoid, backClimbSolenoid;  // Climb pneumatics

  private Timer robotTimer;

  private Joystick driveStickLeft, driveStickRight, armGamepad;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Initialize drivetrain
    leftFront = new CANSparkMax(0, MotorType.kBrushless);
    leftRear = new CANSparkMax(1, MotorType.kBrushless);
    SpeedControllerGroup m_left = new SpeedControllerGroup(leftFront, leftRear);

    rightFront = new CANSparkMax(2, MotorType.kBrushless);
    rightRear = new CANSparkMax(3, MotorType.kBrushless);
    SpeedControllerGroup m_right = new SpeedControllerGroup(rightFront, rightRear);

    driveTrain = new DifferentialDrive(m_left, m_right);

    // Initialize arm
    elevator = new CANSparkMax(4, MotorType.kBrushless);

    clawOpenCloseSolenoid = new DoubleSolenoid(0, 1);
    stopperPiston = new DoubleSolenoid(2, 3);
    frontClimbSolenoid = new DoubleSolenoid(4, 5);
    backClimbSolenoid = new DoubleSolenoid(6, 7);

    // Initialize other miscelanneous objects
    robotTimer = new Timer();

    driveStickLeft = new Joystick(0);
    driveStickRight = new Joystick(1);
    armGamepad = new Joystick(2);

    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // Reset timer, in case it started for some reason
    robotTimer.stop();
    robotTimer.reset();
    robotTimer.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (driveStickLeft.getRawButton(1)||((driveStickLeft.getRawButton(1)&&driveStickRight.getRawButton(1))&&Math.abs(driveStickLeft.getThrottle())>Math.abs(driveStickRight.getThrottle())))
    {
      double y = driveStickLeft.getY() * driveStickLeft.getThrottle();
      driveTrain.arcadeDrive(y, 0);
    } else if (driveStickRight.getRawButton(1)||((driveStickLeft.getRawButton(1)&&driveStickRight.getRawButton(1))&&Math.abs(driveStickRight.getThrottle())>Math.abs(driveStickLeft.getThrottle())))
    {
      double y = driveStickRight.getY() * driveStickRight.getThrottle();
      driveTrain.arcadeDrive(y, 0);
    } else
    {
      double left = driveStickLeft.getY() * driveStickLeft.getThrottle();
      double right = driveStickRight.getY() * driveStickRight.getThrottle();
      driveTrain.tankDrive(left, right);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
