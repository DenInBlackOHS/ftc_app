package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    /* local OpMode members. */
    HardwareMap hwMap  =  null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmnibot(){

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

        leftMotorFore.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotorFore.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors

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

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

