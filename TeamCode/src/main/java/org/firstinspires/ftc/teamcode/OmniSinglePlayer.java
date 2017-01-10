package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: Single Player", group ="TeleOp")

public class OmniSinglePlayer extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    private boolean aLatched = false;
    private boolean shoot = false;
    private double shootSpeed = HardwareOmnibot.HIGH_SHOOT_SPEED;
    private double driverAngle = 0.0;

    // This function allows us to test the motors correspond to what we think they are
    // and that they spin the direction we think they should.

    @Override
    public void start()
    {
        // Executes when start is pressed before loop starts getting called
        // Might want to reset the Gyro here.
        robot.calibrateGyro();
    }

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        double yPower;
        double xPower;
        double spin;

        boolean sweeper = gamepad2.x;
        boolean liftReverse = gamepad2.b;
        boolean aPressed = gamepad2.a;

        yPower = -gamepad2.left_stick_y;
        xPower = gamepad2.left_stick_x;
        spin = gamepad2.right_stick_x;

        boolean motorTest = true;



            if (gamepad1.y) {
                // The driver presses X, then uses the left joystick to say what angle the robot
                // is aiming.  This will calculate the values as long as X is pressed, and will
                // not drive the robot using the left stick.  Once X is released, it will use the
                // final calculated angle and drive with the left stick.  Button should be released
                // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
                // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
                driverAngle = -toDegrees(atan2(xPower, yPower));
                if (driverAngle < 0) {
                    driverAngle += 360;
                }
                xPower = 0.0;
                yPower = 0.0;
                spin = 0.0;
            }

            robot.drive(xPower, yPower, spin, driverAngle);


        //sweeper
        if (sweeper) {
            robot.setLiftMotorPower(HardwareOmnibot.LIFT_SPEED);
            robot.setSweeperMotorPower(HardwareOmnibot.SWEEP_SPEED);
        } else if (liftReverse) {
            robot.setLiftMotorPower(-HardwareOmnibot.LIFT_SPEED);
        } else {
            robot.setLiftMotorPower(0.0);
            robot.setSweeperMotorPower(0.0);
        }

        //shooter
        if (aPressed) {
            if(!aLatched) {
                aLatched = true;
                shoot = !shoot;
            }
        } else {
            aLatched  =  false;
        }

        if (gamepad2.dpad_left) {
            shootSpeed = HardwareOmnibot.LOW_SHOOT_SPEED;
        } else if (gamepad2.dpad_up) {
            shootSpeed = HardwareOmnibot.HIGH_SHOOT_SPEED;
        } else if (gamepad2.dpad_right) {
            shootSpeed = HardwareOmnibot.MID_HIGH_SHOOT_SPEED;
        } else if (gamepad2.dpad_down) {
            shootSpeed = HardwareOmnibot.MID_LOW_SHOOT_SPEED;
        }

        if(shoot) {
            robot.setShooterSpeed(HardwareOmnibot.HIGH_SHOOT_SPEED);
        } else {
            robot.setShooterSpeed(0.0);
        }

        telemetry.addData("Red Detected: ", robot.colorSensorFront.red());
        telemetry.addData("Blue Detected: ", robot.colorSensorFront.blue());
        telemetry.addData("ODS Detected: ", robot.readOds());
        telemetry.addData("dpad_left: ", gamepad2.dpad_left);
        telemetry.addData("dpad_up: ", gamepad2.dpad_up);
        telemetry.addData("dpad_right: ", gamepad2.dpad_right);
        telemetry.addData("dpad_down: ", gamepad2.dpad_down);
        telemetry.addData("shoot Speed: ", shootSpeed);
        telemetry.addData("Shoot Motor1: ", robot.shootMotor1.getPower());
        telemetry.addData("Shoot Motor1 Enc: ", robot.shootMotor1.getCurrentPosition());
        telemetry.addData("Shoot Motor2: ", robot.shootMotor2.getPower());
        telemetry.addData("Shoot Motor2 Enc: ", robot.shootMotor1.getCurrentPosition());
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("spin: ", spin);
        telemetry.addData("sweeper: ", sweeper);
        telemetry.addData("LeftMotorFore: ", robot.leftMotorFore.getPower());
        telemetry.addData("LeftMotorForeEnc: ", robot.leftMotorFore.getCurrentPosition());
        telemetry.addData("rightMotorFore: ", robot.rightMotorFore.getPower());
        telemetry.addData("rightMotorForeEnc: ", robot.rightMotorFore.getCurrentPosition());
        telemetry.addData("rightMotorRear: ", robot.rightMotorRear.getPower());
        telemetry.addData("rightMotorRearEnc: ", robot.rightMotorRear.getCurrentPosition());
        telemetry.addData("leftMotorRear: ", robot.leftMotorRear.getPower());
        telemetry.addData("leftMotorRearEnc: ", robot.leftMotorRear.getCurrentPosition());
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", robot.readGyro());
        updateTelemetry(telemetry);
    }
}
