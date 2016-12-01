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
        robot.leftMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.init(hardwareMap);

        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerInch = (encoderClicksPerRev) / (Math.PI * myWheelSize);
    }


    public void driveForward(double speed, double distance, int maxTime) {

        int sleepTime = 0;
        int position = robot.leftMotorRear.getCurrentPosition();
        int clicksForDistance = position + (int) (distance * clicksPerInch);

        /*robot.leftMotorFore.setTargetPosition(clicksForDistance);
        robot.leftMotorRear.setTargetPosition(clicksForDistance);
        robot.rightMotorFore.setTargetPosition(clicksForDistance);
        robot.rightMotorRear.setTargetPosition(clicksForDistance);

        robot.leftMotorFore.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFore.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        do {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + clicksForDistance;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Since this is a driveForward function, the Y Axis is the only important axis
            robot.drive(0.0, speed, 0.0, 0.0);
//            robot.leftMotorFore.setPower(speed);
//            robot.rightMotorFore.setPower(-speed);
//            robot.leftMotorRear.setPower(speed);
//            robot.rightMotorRear.setPower(-speed);
            updateTelemetry(telemetry);

            position = robot.leftMotorRear.getCurrentPosition();
            sleep(50);
            sleepTime += 50;
            if(isStopRequested())
            {
                // If stop has been requested, break out of the while loop.
                break;
            }
        } while ((position < clicksForDistance) && (sleepTime < maxTime));

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