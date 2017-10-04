package org.usfirst.frc.team263.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;

public class TankDrive {
    private SpeedController mFrontRight, mBackRight, mFrontLeft, mBackLeft;
    private double SPEED_CONSTANT;

    /**
     * Construct instance of drivebase code with proper motor controller
     * addresses.
     * 
     * @param frontRight
     *            Front right motor controller
     * @param backRight
     *            Back right motor controller
     * @param frontLeft
     *            Front left motor controller
     * @param backLeft
     *            Back left motor controller
     * @param k
     *            Proportionality constant for speed
     */
    public TankDrive(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft,
            SpeedController backLeft, double k) {
        mFrontRight = frontRight;
        mBackRight = backRight;
        mFrontLeft = frontLeft;
        mBackLeft = backLeft;
        SPEED_CONSTANT = k;
    }

    /**
     * Construct instance of drivebase code with proper motor controller
     * addresses.
     * 
     * @param frontRight
     *            Front right motor controller
     * @param backRight
     *            Back right motor controller
     * @param frontLeft
     *            Front left motor controller
     * @param backLeft
     *            Back left motor controller
     */
    public TankDrive(SpeedController frontRight, SpeedController backRight, SpeedController frontLeft,
            SpeedController backLeft) {
        this(frontRight, backRight, frontLeft, backLeft, 1.0f);
    }

    /**
     * Method to drive robot, updating motor speeds based off of user input.
     * 
     * @param controller
     *            Xbox Controller to read input from.
     */
    public void drive(XboxController controller) {
        double leftSpeed = SPEED_CONSTANT * (deadband(controller.getY(GenericHID.Hand.kLeft), 0.1)
                + deadband(controller.getTriggerAxis(GenericHID.Hand.kLeft), 0.1)
                - deadband(controller.getTriggerAxis(GenericHID.Hand.kRight), 0.1));

        double rightSpeed = SPEED_CONSTANT * (deadband(controller.getY(GenericHID.Hand.kRight), 0.1)
                - deadband(controller.getTriggerAxis(GenericHID.Hand.kLeft), 0.1)
                + deadband(controller.getTriggerAxis(GenericHID.Hand.kRight), 0.1));

        mFrontRight.set(rightSpeed);
        mBackRight.set(rightSpeed);
        mFrontLeft.set(leftSpeed);
        mBackLeft.set(leftSpeed);
    }

    /**
     * Method to drive robot, updating motor speeds based off of user input.
     * 
     * @param leftSpeed
     *            Speed to spin left wheels
     * @param rightSpeed
     *            Speed to spin right wheels
     */
    public void drive(double leftSpeed, double rightSpeed) {
        mFrontRight.set(rightSpeed);
        mBackRight.set(rightSpeed);
        mFrontLeft.set(leftSpeed);
        mBackLeft.set(leftSpeed);
    }

    /**
     * Creates an artificial deadband on given values.
     * 
     * @param value
     *            Value to place deadband on
     * @param threshhold
     *            Minimum threshhold for preservation of value
     * @return value if |v| > t, 0 otherwise
     */
    public double deadband(double value, double threshhold) {
        return Math.abs(value) >= threshhold ? value : 0.0f;
    }
}
