package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ethan on 10/23/2016.
 */
public class OmniAutoClass extends LinearOpMode {

    HardwareOmnibot robot = new HardwareOmnibot();

    @Override
    public void runOpMode() throws InterruptedException {
    }

    private double myWheelSize;
    private double myMotorRatio;
    private static final double encoderClicksPerRev = 560;
    private static double clicksPerInch = encoderClicksPerRev / (Math.PI * 4);

    /*
     * Sets up the parameters of the robot to use in our functions
     *
     * wheelSize = Diameter of the wheel in inches
     */
    public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
        robot.init(hardwareMap);
        robot.leftMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerInch = (encoderClicksPerRev) / (Math.PI * myWheelSize);
    }


    public void driveForward(double speed, double distance, int maxTime) {

        int sleepTime = 0;
        int position = robot.leftMotorRear.getCurrentPosition();
        int clicksForDistance = position + (int) (distance * clicksPerInch);

        while ((position < clicksForDistance) && (sleepTime < maxTime))
        {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + clicksForDistance;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Since this is a driveForward function, the Y Axis is the only important axis
            robot.drive(0.0, speed, 0.0, 0.0);
            updateTelemetry(telemetry);

            sleep(50);
            sleepTime += 50;
            position = robot.leftMotorRear.getCurrentPosition();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        robot.leftMotorFore.setPower(0);
        robot.rightMotorFore.setPower(0);
        robot.leftMotorRear.setPower(0);
        robot.rightMotorRear.setPower(0);
    }

    public void driveForwardOnHeading(double speed, double distance, int maxTime) {

        int sleepTime = 0;
        int position = robot.leftMotorRear.getCurrentPosition();
        int clicksForDistance = position + (int) (distance * clicksPerInch);
        double gyroReading = robot.readGyro();
        double holdAngle = gyroReading;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 2;
        final double ROTATE_SPEED = 0.1;
        double rotateSpeed = 0.0;

        while ((position < clicksForDistance) && (sleepTime < maxTime))
        {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + clicksForDistance;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);
            telemetry.addData("Sleep Time: ", sleepTime);

            gyroReading = robot.readGyro();
            deltaAngle = gyroReading - holdAngle;

            if(Math.abs(deltaAngle) > SAME_ANGLE)
            {
                if(deltaAngle > 0.0)
                {
                    rotateSpeed = ROTATE_SPEED;
                }
                else
                {
                    rotateSpeed = - ROTATE_SPEED;
                }
            }
            else
            {
                rotateSpeed = 0.0;
            }
            // Since this is a driveForward function, the Y Axis is the only important axis
            robot.drive(0.0, speed, rotateSpeed, 0.0);
            updateTelemetry(telemetry);

            sleep(50);
            sleepTime += 50;
            position = robot.leftMotorRear.getCurrentPosition();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        robot.leftMotorFore.setPower(0);
        robot.rightMotorFore.setPower(0);
        robot.leftMotorRear.setPower(0);
        robot.rightMotorRear.setPower(0);
    }

    public void rotateRobot(double speed, double angle, int maxTime) {

        int sleepTime = 0;
        double gyroReading = robot.readGyro();
        double deltaAngle = gyroReading - angle;
        final double SAME_ANGLE = 2;
        double rotateSpeed = 0.0;

        while (Math.abs(deltaAngle) > SAME_ANGLE && (sleepTime < maxTime)) {
            String myTelemetry = "Current Angle: " + gyroReading + " Destination Angle: " + angle;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Sleep Time: ", sleepTime);

            if(deltaAngle > 0.0)
            {
                // Positive angle, need to rotate right
                rotateSpeed = speed;
            }
            else
            {
                // Negative angle, need to rotate left;
                rotateSpeed = - speed;
            }
            telemetry.addData("Rotate Speed: ", rotateSpeed);
            robot.drive(0.0, 0.0, rotateSpeed, 0.0);
            updateTelemetry(telemetry);

            sleep(50);
            sleepTime += 50;
            gyroReading = robot.readGyro();
            deltaAngle = gyroReading - angle;
            if(isStopRequested())
            {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        robot.leftMotorFore.setPower(0);
        robot.rightMotorFore.setPower(0);
        robot.leftMotorRear.setPower(0);
        robot.rightMotorRear.setPower(0);
    }

    public void shoot(double speed, int fireTime) {

        int sleepTime = 0;

        telemetry.addData("Setting Power: ", speed);
        telemetry.addData("Sleep Time: ", sleepTime);
        robot.shootMotor1.setPower(speed);
        robot.shootMotor2.setPower(speed);

        // Allow the shooter motors to come up to speed
        sleep(200);

        // Lift the balls to the shooter
        robot.liftMotor.setPower(.75);
        robot.armMotor.setPower(.75);

        updateTelemetry(telemetry);

        // Let things happen
        sleep(fireTime);

        robot.shootMotor1.setPower(0);
        robot.shootMotor2.setPower(0);
        robot.liftMotor.setPower(0);
        robot.armMotor.setPower(0);
    }
}