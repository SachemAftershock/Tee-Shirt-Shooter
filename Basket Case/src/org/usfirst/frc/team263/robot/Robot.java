package org.usfirst.frc.team263.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Sachem Aftershock's TShirt Shooter Code
 * @author Dan Waxman
 * @author Rohan Bapat
 * @since 01/29/17
 */
public class Robot extends SampleRobot {
	CANTalon backLeft, frontLeft, backRight, frontRight;
	MecanumDrive drivebase;
	AHRS gyro;
	Solenoid leftCannon, rightCannon;
	XboxController drivePad;
	Timer leftTimer, rightTimer;
	boolean safetyOn;
	
	public Robot() {
		backLeft = new CANTalon(3);
		frontLeft = new CANTalon(4);
		backRight = new CANTalon(5);
		frontRight = new CANTalon(6);
		gyro = new AHRS(SerialPort.Port.kMXP);
		drivebase = new MecanumDrive(frontRight, backRight, frontLeft, backLeft, gyro);
		leftCannon = new Solenoid(5);
		rightCannon = new Solenoid(6);
		drivePad = new XboxController(0);
		leftTimer = new Timer();
		rightTimer = new Timer();
		LEDStrip.sendColor(LEDStrip.LEDMode.eRainbow);
		safetyOn = false;
	}

	@Override
	public void operatorControl() {
		while (isEnabled() && isOperatorControl()) {
			// Close solenoids after 1/10 seconds if they're open
			if (leftTimer.get() > 0.1) {
				leftTimer.stop();
				leftTimer.reset();
				leftCannon.set(false);
			}
			if (rightTimer.get() > 0.1) {
				rightTimer.stop();
				rightTimer.reset();
				rightCannon.set(false);
			}
			
			// Have X Button as a safety
			// If X is pressed, use left and right bumpers to set off cannons
			if (drivePad.getXButton()) {
				safetyOn = true;
				if (drivePad.getBumper(Hand.kLeft)) {
					leftCannon.set(true);
					leftTimer.start();
				} 
				if (drivePad.getBumper(Hand.kRight)) {
					rightCannon.set(true);
					rightTimer.start();
				}
			} else {
				safetyOn = false;
			}
			
			if (safetyOn) {
				LEDStrip.sendColor(LEDStrip.LEDMode.eRed);
			} else {
				LEDStrip.sendColor(LEDStrip.LEDMode.eRainbow);
			}
		}
	}
}
