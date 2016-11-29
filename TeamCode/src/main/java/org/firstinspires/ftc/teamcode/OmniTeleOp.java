package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;
import static java.lang.Math.max;

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

    private boolean aLatched = false;
    private boolean shoot = false;
    private double shootSpeed = 0.90;

    @Override
    public void loop() {
        //left joystick is for moving
        //right joystick is for rotation
        double yPower;
        double xPower;
        double spinright;

        boolean arm = gamepad2.x;
        boolean armReverse = gamepad2.b;
        boolean aPressed = gamepad2.a;

        yPower = gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spinright = gamepad1.right_stick_x * .2;

        boolean leftSpeed = gamepad2.dpad_left;
        boolean upSpeed = gamepad2.dpad_up;
        boolean downSpeed = gamepad2.dpad_down;
        boolean rightSpeed = gamepad2.dpad_right;

        double leftFrontAngle = toRadians(45.0);
        double rightFrontAngle = toRadians(135.0);
        double leftRearAngle = toRadians(-45.0);
        double rightRearAngle = toRadians(-135.0);

        double LFpower = (xPower * cos(leftFrontAngle) + yPower * sin(leftFrontAngle));
        double LRpower = (xPower * cos(leftRearAngle) + yPower * sin(leftRearAngle));
        double RFpower = (xPower * cos(rightFrontAngle) + yPower * sin(rightFrontAngle));
        double RRpower = (xPower * cos(rightRearAngle) + yPower * sin(rightRearAngle));

          /*double max = max(max(LFpower, LRpower),
                         max(RFpower, RRpower));

        LFpower /= max;
        RFpower /= max;
        RFpower /= max;
        RRpower /= max;    */

        if(abs(yPower) > .1 || abs(xPower) > .1 || abs(spinright) > .02) {

            robot.leftMotorFore.setPower(LFpower/sqrt(2) + spinright);
            robot.rightMotorFore.setPower(RFpower/sqrt(2) - spinright);
            robot.leftMotorRear.setPower(LRpower/sqrt(2) - spinright);
            robot.rightMotorRear.setPower(RRpower/sqrt(2) + spinright);
            /*telemetry.addData("Left Fore Enc: ", robot.leftMotorFore.getCurrentPosition());
            telemetry.addData("Left Rear Enc: ", robot.leftMotorRear.getCurrentPosition());
            telemetry.addData("Right Fore Enc: ", robot.rightMotorFore.getCurrentPosition());
            telemetry.addData("Right Rear Enc: ", robot.rightMotorRear.getCurrentPosition());*/

        }
        else
        {
            robot.leftMotorFore.setPower(0);
            robot.rightMotorFore.setPower(0);
            robot.leftMotorRear.setPower(0);
            robot.rightMotorRear.setPower(0);
        }
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
            shootSpeed = .15;
        } else if (upSpeed) {
            shootSpeed = .6;
        } else if (rightSpeed) {
            shootSpeed = .45;
        } else if (downSpeed) {
            shootSpeed = .3;
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
        telemetry.addData("spinright: ", spinright);
        telemetry.addData("arm: ", arm);
        telemetry.addData("LeftMotorFore: ", robot.leftMotorFore.getPower());
        telemetry.addData("rightMotorFore: ", robot.rightMotorFore.getPower());
        telemetry.addData("rightMotorRear: ", robot.rightMotorRear.getPower());
        telemetry.addData("leftMotorRear: ", robot.leftMotorRear.getPower());
        updateTelemetry(telemetry);
    }
}
