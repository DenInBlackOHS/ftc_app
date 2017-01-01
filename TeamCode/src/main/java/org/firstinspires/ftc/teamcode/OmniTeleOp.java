package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;

/**
 * Created by Ethan on 12/2/2016.
 */

@TeleOp(name="Omni: TeleOp", group ="TeleOp")

public class OmniTeleOp extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Turn off the unused sensors for teleop
        robot.resetGyro();
// TODO: Add back in when not testing sensors.
//        robot.disableColorSensors();
//        robot.disableRangeSensors();
    }

    private boolean aLatched = false;
    private boolean shoot = false;
    private double shootSpeed = HardwareOmnibot.HIGH_SHOOT_SPEED;
    private double driverAngle = 0.0;

    // This function allows us to test the motors correspond to what we think they are
    // and that they spin the direction we think they should.
    private void motorTest(boolean x, boolean y, boolean b, boolean a)
    {
        if(x)
        {
            robot.leftMotorFore.setPower(1.0);
        }
        else
        {
            robot.leftMotorFore.setPower(0.0);
        }
        if(y)
        {
            robot.rightMotorFore.setPower(1.0);
        }
        else
        {
            robot.rightMotorFore.setPower(0.0);
        }
        if(b)
        {
            robot.rightMotorRear.setPower(1.0);
        }
        else
        {
            robot.rightMotorRear.setPower(0.0);
        }
        if(a)
        {
            robot.leftMotorRear.setPower(1.0);
        }
        else
        {
            robot.leftMotorRear.setPower(0.0);
        }
    }

    @Override
    public void start()
    {
        // Executes when start is pressed before loop starts getting called
        // Might want to reset the Gyro here.
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

        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        boolean motorTest = false ;

        if(motorTest) {
            motorTest(gamepad1.x, gamepad1.y, gamepad1.b, gamepad1.a);
        }
        else {
            if (gamepad1.x) {
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
                driverAngle -= robot.readGyro();
                xPower = 0.0;
                yPower = 0.0;
                spin = 0.0;
            }

            robot.drive(xPower, yPower, spin, driverAngle);
        }

        //sweeper
        if (sweeper) {
            robot.sweeperMotor.setPower(.75);
            robot.liftMotor.setPower(.75);
        } else if (liftReverse) {
            robot.liftMotor.setPower(-.75);
        } else {
            robot.sweeperMotor.setPower(0);
            robot.liftMotor.setPower(0);
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
            robot.shootMotor1.setPower(shootSpeed);
            robot.shootMotor2.setPower(shootSpeed);
        } else {
            robot.shootMotor1.setPower(0);
            robot.shootMotor2.setPower(0);
        }

        telemetry.addData("Shoot Speed: ", shootSpeed);
        telemetry.addData("Shoot Motor1: ", robot.shootMotor1.getPower());
        telemetry.addData("Shoot Motor1 Enc: ", robot.shootMotor1.getCurrentPosition());
        telemetry.addData("Shoot Motor2: ", robot.shootMotor2.getPower());
        telemetry.addData("Shoot Motor2 Enc: ", robot.shootMotor1.getCurrentPosition());
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Sweeper: ", sweeper);
        telemetry.addData("LeftMotorFore: ", robot.leftMotorFore.getPower());
        telemetry.addData("LeftMotorForeEnc: ", robot.leftMotorFore.getCurrentPosition());
        telemetry.addData("RightMotorFore: ", robot.rightMotorFore.getPower());
        telemetry.addData("RightMotorForeEnc: ", robot.rightMotorFore.getCurrentPosition());
        telemetry.addData("RightMotorRear: ", robot.rightMotorRear.getPower());
        telemetry.addData("RightMotorRearEnc: ", robot.rightMotorRear.getCurrentPosition());
        telemetry.addData("LeftMotorRear: ", robot.leftMotorRear.getPower());
        telemetry.addData("LeftMotorRearEnc: ", robot.leftMotorRear.getCurrentPosition());
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", robot.readGyro());
        telemetry.addData("Range Front: ", robot.readFrontRangeSensor());
        telemetry.addData("Range Back: ", robot.readBackRangeSensor());
        telemetry.addData("Color Red Front: ", robot.readRedFrontColorSensor());
        telemetry.addData("Color Blue Front: ", robot.readBlueFrontColorSensor());
        telemetry.addData("Color Red Back: ", robot.readRedBackColorSensor());
        telemetry.addData("Color Blue Back: ", robot.readBlueBackColorSensor());
        updateTelemetry(telemetry);
    }
}
