/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


//Import needed classes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Robot extends TimedRobot {

	// Create objects
	private DifferentialDrive driveTrain;	// Drive train
	private Joystick driveStickLeft;
  	
	Ultrasonic ultrasonic = new Ultrasonic(1,0);
	double distance = 0.0, distance_old = distance, turn = 0.0;
	Timer driveTimer;
	boolean timerRunning = false;
	int target = 25;
	TalonSRX arm;

	@Override
	public void robotInit() {	// Run when robot is turned on. Initialize m.controllers, etc.
		ultrasonic.setEnabled(true);
		ultrasonic.setAutomaticMode(true);
		driveTimer = new Timer();
		driveTimer.start();
		// Initialize drive train
		Spark m_rearLeft = new Spark(0);	// Initialize left Sparks
		Spark m_frontLeft = new Spark(1);

		SpeedControllerGroup m_left = new SpeedControllerGroup(m_rearLeft, m_frontLeft);	// Join left Sparks into a group

		Spark m_frontRight = new Spark(2);	// Initialize right Sparks
		Spark m_rearRight = new Spark(3);
		SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);	// Join right Sparks into a group

		driveTrain = new DifferentialDrive(m_left, m_right);	// Create drive train with Spark groups
		driveStickLeft = new Joystick(0);	// Initialize left joystick
		arm = new TalonSRX(0);
	}

	private int speedToTarget(int target, double current) {
		double multiplier = 80 / 6.78;
		int direction = 0;
		boolean outOfDeadZone = Math.abs(target - current) <= 1;
		System.out.println("Distance: " + Math.abs(target - current) + "\tBelow 1: " + outOfDeadZone);
		if (!outOfDeadZone) {
			System.out.println("Target > Current: " + (target > current) + "Target < Current: " + (target < current));
			if (target > current) {
				direction = 1;
			} else if (target < current) {
				direction = -1;
			} else {
				direction = 0;
			}
		} else {
			direction = 0;
		}
		System.out.println("Direction: " + direction);
		double speed = multiplier * direction * Math.sqrt(Math.abs(target - current));
		return (int)speed;
	}

	/*private void armGoToPosition() {
		if (currentPosition != 0 || target != -1) {
			target = positions[currentPosition];
			m_elevator.set(speedToTarget(target, getUltrasonicDistance(ultrasonicSensor)));
		}
	}*/

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putData("Ultrasonic Sensor", ultrasonic);
		SmartDashboard.putNumber("Ultrasonic Distance (in)", ultrasonic.getRangeInches());
		distance = ultrasonic.getRangeInches();
		SmartDashboard.putString("Distance to Target", "Distance: " + (target - ultrasonic.getRangeInches()) + "\tTarget: " + target + "\t Speed: " + speedToTarget(target, ultrasonic.getRangeInches()));
		arm.set(ControlMode.PercentOutput, speedToTarget(target, ultrasonic.getRangeInches()));
	}

	@Override
	public void teleopPeriodic() {
		// autonomousPeriodic();

		boolean button5Pressed = driveStickLeft.getRawButton(5);
		boolean button3Pressed = driveStickLeft.getRawButton(3);
		if (button5Pressed && !button3Pressed) {
			arm.set(ControlMode.PercentOutput, 0.5);
		} else if (!button5Pressed && button3Pressed) {
			arm.set(ControlMode.PercentOutput, -0.5);
		} else {
			arm.set(ControlMode.PercentOutput, 0.0);
		}
	}
}