package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        robot.calibrateGyro();
//        robot.disableColorSensors();
//        robot.disableRangeSensors();
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    private boolean aLatched = false;
    private boolean shoot = false;
    private double shootSpeed = HardwareOmnibot.HIGH_SHOOT_SPEED;
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

        boolean sweeper = gamepad2.x;
        boolean liftReverse = gamepad2.b;
        boolean aPressed = gamepad2.a;

        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        boolean motorTest = false ;

        if (gamepad1.x) {
            // The driver presses X, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as X is pressed, and will
            // not drive the robot using the left stick.  Once X is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            robot.resetGyro();
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

        //sweeper
        if (sweeper) {
            robot.setSweeperMotorPower(HardwareOmnibot.SWEEP_SPEED);
            robot.setLiftMotorPower(HardwareOmnibot.LIFT_SPEED);
        } else if (liftReverse) {
            robot.setLiftMotorPower(-HardwareOmnibot.LIFT_SPEED);
        } else {
            robot.setSweeperMotorPower(0.0);
            robot.setLiftMotorPower(0.0);
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
            robot.setShooterSpeed(shootSpeed);
        } else {
            if(gamepad2.y)
            {
                robot.setShooterSpeed(-0.3);
            }
            else
            {
                robot.setShooterSpeed(0.0);
            }
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
        telemetry.addData("Range Back: ", robot.readBackRangeSensor());
        telemetry.addData("Color Red Front: ", robot.readRedFrontColorSensor());
        telemetry.addData("Color Blue Front: ", robot.readBlueFrontColorSensor());
        telemetry.addData("Color Red Back: ", robot.readRedBackColorSensor());
        telemetry.addData("Color Blue Back: ", robot.readBlueBackColorSensor());
        telemetry.addData("ODS Reading: ", robot.readOds());
        updateTelemetry(telemetry);
    }
}
