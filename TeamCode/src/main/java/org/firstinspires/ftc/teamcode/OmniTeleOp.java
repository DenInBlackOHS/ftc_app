package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;

/**
 * Created by Ethan on 10/2/2016.
 */

@TeleOp(name="Omni: TeleOp", group ="TeleOp")

public class OmniTeleOp extends OpMode {

    HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    private static double highShootSpeed = 0.60;
    private static double midHighShootSpeed = 0.45;
    private static double midLowShootSpeed = 0.30;
    private static double lowShootSpeed = 0.15;
    private boolean aLatched = false;
    private boolean shoot = false;
    private double shootSpeed = highShootSpeed;
    private double driverAngle = 0.0;

    @Override
    public void start()
    {
        // Executes when start is pressed before loop starts getting called
        // Might want to reset the Gyro here.
        robot.resetGyro();
    }

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        double yPower;
        double xPower;
        double spin;

        boolean arm = gamepad2.x;
        boolean armReverse = gamepad2.b;
        boolean aPressed = gamepad2.a;

        yPower = gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = -gamepad1.right_stick_x;

        boolean leftSpeed = gamepad2.dpad_left;
        boolean upSpeed = gamepad2.dpad_up;
        boolean downSpeed = gamepad2.dpad_down;
        boolean rightSpeed = gamepad2.dpad_right;

        if(gamepad1.x)
        {
            // The driver presses X, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as X is pressed, and will
            // not drive the robot using the left stick.  Once X is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.
            driverAngle = toDegrees(atan2(xPower, yPower));
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

        robot.drive(xPower, yPower, spin, driverAngle);

        /*telemetry.addData("Left Fore Enc: ", robot.leftMotorFore.getCurrentPosition());
        telemetry.addData("Left Rear Enc: ", robot.leftMotorRear.getCurrentPosition());
        telemetry.addData("Right Fore Enc: ", robot.rightMotorFore.getCurrentPosition());
        telemetry.addData("Right Rear Enc: ", robot.rightMotorRear.getCurrentPosition());*/
        //arm
        if (arm) {
            robot.armMotor.setPower(.75);
            robot.liftMotor.setPower(.75);
        } else if (armReverse) {
            robot.armMotor.setPower(-.75);
            robot.liftMotor.setPower(-.75);
        } else {
            robot.armMotor.setPower(0);
            robot.liftMotor.setPower(0);
        }

        //shooter
        if (aPressed) {
            if(!aLatched) {
                aLatched = true;
                if (shoot) {
                    shoot = false;
                } else {
                    shoot = true;
                }
            }
        } else {
            aLatched  =  false;
        }

        if (leftSpeed) {
            shootSpeed = lowShootSpeed;
        } else if (upSpeed) {
            shootSpeed = highShootSpeed;
        } else if (rightSpeed) {
            shootSpeed = midHighShootSpeed;
        } else if (downSpeed) {
            shootSpeed = midLowShootSpeed;
        }

        if(shoot) {
            robot.shootMotor1.setPower(shootSpeed);
            robot.shootMotor2.setPower(shootSpeed);
            telemetry.addData("shoot Speed: ", shootSpeed);
        } else {
            robot.shootMotor1.setPower(0);
            robot.shootMotor2.setPower(0);
        }

        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("spin: ", spin);
        telemetry.addData("arm: ", arm);
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
