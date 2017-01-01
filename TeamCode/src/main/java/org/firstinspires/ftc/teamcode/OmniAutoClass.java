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

    // Default to 4" wheels
    private static double myWheelSize = 4.0;
    // Default to 40:1 motors
    private static double myMotorRatio = 40.0;

    // 20:1 motor = 560
    // 40:1 motor = 1120
    // 60:1 motor = 1680
    private static final double encoderClicksPerRev = 28;
    private static double clicksPerInch = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize);
    public static final float MM_PER_INCH = 25.4f;


    /*
     * Sets up the parameters of the robot to use in our functions
     *
     * wheelSize = Diameter of the wheel in inches
     */
    public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
        robot.init(hardwareMap);
        robot.enableColorSensors();
        robot.enableRangeSensors();

        robot.resetDriveEncoders();
        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerInch = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize);
    }

    public void allDriveStop()
    {
        robot.leftMotorFore.setPower(0);
        robot.rightMotorFore.setPower(0);
        robot.leftMotorRear.setPower(0);
        robot.rightMotorRear.setPower(0);
    }

    public void driveForward(double speed, double distance, int maxTime) {

        int sleepTime = 0;
        final int deltaSleep = 50;
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

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            position = robot.leftMotorRear.getCurrentPosition();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        allDriveStop();
    }

    public boolean reachedClickPosition(int position, int destination, double speed)
    {
        boolean result = false;
        if(speed > 0)
        {
            if(position >= destination) {
                result = true;
            }
        }
        else
        {
            if(position <= destination) {
                result = true;
            }
        }
        return result;
    }

    public void driveForwardOnHeading(double speed, double distance, int maxTime) {

        //temporary fix until we understand
        double correctedDistance = distance / 1.75;

        int sleepTime = 0;
        final int deltaSleep = 50;
        int position = robot.leftMotorRear.getCurrentPosition();
        int finalEncoderValue;

        if (speed < 0) {
            finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
        } else {
            finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
        }

        double gyroReading = robot.readGyro();
        double holdAngle = gyroReading;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 2;
        final double ROTATE_SPEED = 0.1;
        double rotateSpeed = 0.0;

        while ((!reachedClickPosition(position, finalEncoderValue, speed) && (sleepTime < maxTime)))
        {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + finalEncoderValue;
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
            robot.drive(0.0, speed, rotateSpeed, -gyroReading);
            updateTelemetry(telemetry);

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            position = robot.leftMotorRear.getCurrentPosition();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        allDriveStop();
    }

    public void rotateRobot(double speed, double angle, int maxTime) {

        int sleepTime = 0;
        final int deltaSleep = 50;
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

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            gyroReading = robot.readGyro();
            deltaAngle = gyroReading - angle;
            if(isStopRequested())
            {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        allDriveStop();
    }

    public void shoot(double speed, int fireTime) {
        int shooterWarmUp = 1000;
        int sleepTime = 0;

        telemetry.addData("Setting Power: ", speed);
        telemetry.addData("Sleep Time: ", sleepTime);
        robot.shootMotor1.setPower(speed);
        robot.shootMotor2.setPower(speed);

        // Allow the shooter motors to come up to speed
        sleep(shooterWarmUp);

        // Lift the balls to the shooter
        robot.liftMotor.setPower(.75);
        robot.sweeperMotor.setPower(.75);

        updateTelemetry(telemetry);

        // Let things happen, time passed in is total time, not just the time
        // to lift and fire the balls.
        sleep(fireTime - shooterWarmUp);

        robot.shootMotor1.setPower(0);
        robot.shootMotor2.setPower(0);
        robot.liftMotor.setPower(0);
        robot.sweeperMotor.setPower(0);
    }

    /**
     *
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledDeceleration(double distanceToTravelMm, double maxSpeed)
    {
        final double fastDistance = 150.0;
        final double mediumDistance = 75.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;

        double result = 0.0;

        if(distanceToTravelMm > fastDistance)
        {
            result = maxSpeed;
        }
        else if(distanceToTravelMm > mediumDistance)
        {
            result = maxSpeed / mediumDivider;
        }
        else
        {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     *
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationMm(double distanceToTravelMm, double maxSpeed)
    {
        final double fastDistance = 75.0;
        final double mediumDistance = 50.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;

        double result = 0.0;

        if(distanceToTravelMm > fastDistance)
        {
            result = maxSpeed;
        }
        else if(distanceToTravelMm > mediumDistance)
        {
            result = maxSpeed / mediumDivider;
        }
        else
        {
            result = maxSpeed / slowDivider;
        }

        return result;
    }

    /**
     *
     * @param maxSpeed - The speed to use when going large distances
     * @param distanceFromWallMm - The distance to make the robot parallel to the wall in mm
     * @param timeout - The maximum amount of time to wait until giving up
     */
    public void moveToWall(double maxSpeed, double distanceFromWallMm, int timeout)
    {
        double allowedDistanceError = 10;
        int sleepTime = 0;
        final int deltaSleep = 50;
        double frontDistance = robot.readFrontRangeSensor();
        double backDistance = robot.readBackRangeSensor();
        double frontDelta = frontDistance - distanceFromWallMm;
        double backDelta = backDistance - distanceFromWallMm;
        double frontError = Math.abs(frontDelta);
        double backError = Math.abs(backDelta);
        double frontBackDiff = Math.abs(frontDistance - backDistance);
        double speed = 0.0;
        double rotateSpeed = 0.0;

        String myTelemetry = "Front Distance: " + frontDistance + " Back Distance: " + backDistance;
        telemetry.addLine(myTelemetry);
        telemetry.addData("Max Power: ", maxSpeed);
        telemetry.addData("Sleep Time: ", sleepTime);

        while(((frontError > allowedDistanceError) || (backError > allowedDistanceError)) && (sleepTime < timeout))
        {
            // Calculate the linear driving
            if((frontDistance < distanceFromWallMm) && (backDistance < distanceFromWallMm))
            {
                // Drive away from wall
                speed = controlledDeceleration((frontError + backError) / 2.0, maxSpeed);
            } else if ((frontDistance > distanceFromWallMm) && (backDistance > distanceFromWallMm))
            {
                // Drive towards wall
                speed = controlledDeceleration((frontError + backError) / 2.0, -maxSpeed);
            }
            else
            {
                // We just need to rotate
                speed = 0.0;
            }

            // Calculate the rotational driving
            if(frontBackDiff > allowedDistanceError)
            {
                if(frontDelta > 0.0)
                {
                    // Rotate CCW
                    rotateSpeed = controlledRotationMm(frontBackDiff, 0.4);
                }
                else
                {
                    // Rotate CW
                    rotateSpeed = controlledRotationMm(frontBackDiff, -0.4);
                }
            }
            else
            {
                rotateSpeed = 0.0;
            }

            // Drive in the X direction should be the same as driving left/right
            robot.drive(speed, 0.0, rotateSpeed, 0.0);

            // Let things happen
            sleep(deltaSleep);
            sleepTime += deltaSleep;

            // Get new readings
            frontDistance = robot.readFrontRangeSensor();
            backDistance = robot.readBackRangeSensor();
            frontDelta = frontDistance - distanceFromWallMm;
            backDelta = backDistance - distanceFromWallMm;
            frontError = Math.abs(frontDelta);
            backError = Math.abs(backDelta);
            frontBackDiff = Math.abs(frontDistance - backDistance);

            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }
    }

    public void endAuto() {
        robot.disableRangeSensors();
        robot.disableColorSensors();
    }
}