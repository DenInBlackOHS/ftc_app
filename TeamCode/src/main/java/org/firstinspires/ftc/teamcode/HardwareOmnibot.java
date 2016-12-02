package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Thread;

import static java.lang.Math.*;

/**
 *Created by Ethan
 */
public class HardwareOmnibot
{
    /* Public OpMode members. */
    public DcMotor leftMotorFore = null;
    public DcMotor rightMotorFore = null;
    public DcMotor leftMotorRear = null;
    public DcMotor rightMotorRear = null;
    public DcMotor armMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor shootMotor1 = null;
    public DcMotor shootMotor2 = null;
    public Servo buttonPush = null;
    public ModernRoboticsI2cGyro robotGyro = null;

    // Might have to lower from max 2800
    public static final int encoderClicksPerSecond = 2800;

    /* local OpMode members. */
    HardwareMap hwMap  =  null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmnibot(){

    }

    public void initGyro()
    {
        // Init Gyro Code
        robotGyro = hwMap.get(ModernRoboticsI2cGyro.class, "robot_gyro");
        robotGyro.setI2cAddress(I2cAddr.create8bit(0x20));
    }

    public void resetGyro()
    {
        // Reset Gyro Code
        robotGyro.calibrate();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e){}
    }

    public double readGyro()
    {
        // Read Gyro Code
        return (double)robotGyro.getHeading();
    }

    // xPower: -1.0 to 1.0 power in the X axis
    // yPower: -1.0 to 1.0 power in the Y axis
    // spin: -1.0 to 1.0 power to spin, reduced to 20%
    public void drive(double xPower, double yPower, double spin, double angleOffset)
    {
        // Read Gyro Angle Here
        double reducedSpin = spin * 0.2;
        double gyroAngle = readGyro() + angleOffset;
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(315.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(225.0 + gyroAngle);

        if(abs(yPower) < 0.1)
        {
            yPower = 0.0;
        }
        if(abs(xPower) < 0.1)
        {
            xPower = 0.0;
        }
        if(abs(reducedSpin) < 0.02)
        {
            reducedSpin = 0.0;
        }

        double LFpower = (xPower * cos(leftFrontAngle) + yPower * sin(leftFrontAngle))/sqrt(2) + reducedSpin;
        double LRpower = (xPower * cos(leftRearAngle) + yPower * sin(leftRearAngle))/sqrt(2) + reducedSpin;
        double RFpower = (xPower * cos(rightFrontAngle) + yPower * sin(rightFrontAngle))/sqrt(2) + reducedSpin;
        double RRpower = (xPower * cos(rightRearAngle) + yPower * sin(rightRearAngle))/sqrt(2) + reducedSpin;

        double maxPower = max(1.0, max(max(LFpower, LRpower),
                max(RFpower, RRpower)));

        LFpower /= maxPower;
        RFpower /= maxPower;
        RFpower /= maxPower;
        RRpower /= maxPower;

        leftMotorFore.setPower(LFpower);
        rightMotorFore.setPower(RFpower);
        leftMotorRear.setPower(LRpower);
        rightMotorRear.setPower(RRpower);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFore = hwMap.dcMotor.get("MotorLF");
        rightMotorFore  = hwMap.dcMotor.get("MotorRF");
        leftMotorRear = hwMap.dcMotor.get("MotorLR");
        rightMotorRear = hwMap.dcMotor.get("MotorRR");
        armMotor = hwMap.dcMotor.get("ArmMotor");
        liftMotor = hwMap.dcMotor.get("LiftMotor");
        shootMotor1 = hwMap.dcMotor.get("Shoot1");
        shootMotor2 = hwMap.dcMotor.get("Shoot2");
        buttonPush = hwMap.servo.get("ServoButton");

        leftMotorFore.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFore.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.FORWARD);// Set to REVERSE if using AndyMark motors

        armMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        shootMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotorFore.setPower(0);
        rightMotorFore.setPower(0);
        leftMotorRear.setPower(0);
        rightMotorRear.setPower(0);
        armMotor.setPower(0);
        liftMotor.setPower(0);
        shootMotor1.setPower(0);
        shootMotor2.setPower(0);

        leftMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotorFore.setMaxSpeed(encoderClicksPerSecond);
        rightMotorFore.setMaxSpeed(encoderClicksPerSecond);
        leftMotorRear.setMaxSpeed(encoderClicksPerSecond);
        rightMotorRear.setMaxSpeed(encoderClicksPerSecond);
        shootMotor1.setMaxSpeed(encoderClicksPerSecond);
        shootMotor2.setMaxSpeed(encoderClicksPerSecond);

        initGyro();
        // Define and initialize ALL installed servos.
        //leftClaw = hwMap.servo.get("left_hand");
        //rightClaw = hwMap.servo.get("right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

