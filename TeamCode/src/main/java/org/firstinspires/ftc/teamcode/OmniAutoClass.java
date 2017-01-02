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
        robot.calibrateGyro();

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

    private double deltaAngle(double angle, double gyroReading)
    {
        double result = gyroReading - angle;

        if(result < 0.0)
        {
            result += 360.0;
        }

        return result;
    }

    public void rotateRobotToAngle(double speed, double angle, int maxTime) {
        double gyroReading = robot.readGyro();
        // Determine the delta angle, then call rotateRobot
        double deltaAngle = angle - gyroReading;

        rotateRobot(speed, deltaAngle, maxTime);
    }

    /**
     *
     * @param speed - Max speed to rotate
     * @param angle - Angle in degrees to rotate.  Positive right, negative left
     * @param maxTime - Timeout to quit trying
     */
    public void rotateRobot(double speed, double angle, int maxTime) {

        int sleepTime = 0;
        final int deltaSleep = 50;
        double gyroReading = robot.readGyro();
        double targetAngle = gyroReading + angle;
        double angleRemaining = 0.0;
        final double SAME_ANGLE = 5;
        double rotateSpeed = 0.0;

        // We won't do circles with this function, just minimum rotation
        while(targetAngle > 360.0)
        {
            targetAngle -= 360.0;
        }
        while(targetAngle < 0.0)
        {
            targetAngle += 360.0;
        }

        angleRemaining = deltaAngle(targetAngle, gyroReading);
        while (angleRemaining > SAME_ANGLE && (sleepTime < maxTime)) {
            String myTelemetry = "Current Angle: " + gyroReading + " Destination Angle: " + targetAngle;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Sleep Time: ", sleepTime);

            if(angle > 0.0)
            {
                // Negative angle, need to rotate right
                rotateSpeed = - controlledRotationAngle(angleRemaining, speed);
            }
            else
            {
                // Positive angle, need to rotate left;
                rotateSpeed = controlledRotationAngle(angleRemaining, speed);
            }
            telemetry.addData("Rotate Speed: ", rotateSpeed);
            robot.drive(0.0, 0.0, rotateSpeed, 0.0);
            updateTelemetry(telemetry);

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            gyroReading = robot.readGyro();
            angleRemaining = deltaAngle(targetAngle, gyroReading);
            if(isStopRequested())
            {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        allDriveStop();
    }

    public void shoot(int fireTime) {
        int sleepTime = 0;

        telemetry.addData("Sleep Time: ", sleepTime);

        // Lift the balls to the shooter
        robot.liftMotor.setPower(.75);
        robot.sweeperMotor.setPower(.75);

        updateTelemetry(telemetry);

        // Let things happen, time passed in is total time, not just the time
        // to lift and fire the balls.
        sleep(fireTime);

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
        final double fastDistance = 100.0;
        final double mediumDistance = 50.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 3.0;

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
     * @param angleToTravel - How far we are traveling in degrees
     * @param maxSpeed - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledRotationAngle(double angleToTravel, double maxSpeed)
    {
        final double fastAngle = 60.0;
        final double mediumAngle = 45.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;

        double result = 0.0;

        if(angleToTravel > fastAngle)
        {
            result = maxSpeed;
        }
        else if(angleToTravel > mediumAngle)
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
        double maxSpeedAbs = Math.abs(maxSpeed);
        double frontDistance = 0.0;
        double backDistance = 0.0;
        double frontDelta = 0.0;
        double backDelta = 0.0;
        double frontError = 0.0;
        double backError = 0.0;
        double frontBackDiff = 0.0;
        double speed = 0.0;
        double rotateSpeed = 0.0;

        frontDistance = robot.readFrontRangeSensor();
        backDistance = robot.readBackRangeSensor();
        frontDelta = frontDistance - distanceFromWallMm;
        backDelta = backDistance - distanceFromWallMm;
        frontError = Math.abs(frontDelta);
        backError = Math.abs(backDelta);
        frontBackDiff = Math.abs(frontDistance - backDistance);

        while(((frontError > allowedDistanceError) || (backError > allowedDistanceError)) && (sleepTime < timeout))
        {
            String myTelemetry = "Front Distance: " + frontDistance + " Back Distance: " + backDistance;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Max Power: ", maxSpeedAbs);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Calculate the linear driving
            if((frontDistance < distanceFromWallMm) && (backDistance < distanceFromWallMm))
            {
                // Drive away from wall
                speed = controlledDeceleration((frontError + backError) / 2.0, maxSpeedAbs);
            } else if ((frontDistance > distanceFromWallMm) && (backDistance > distanceFromWallMm))
            {
                // Drive towards wall
                speed = controlledDeceleration((frontError + backError) / 2.0, -maxSpeedAbs);
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
                    rotateSpeed = controlledRotationMm(frontBackDiff, 0.2);
                }
                else
                {
                    // Rotate CW
                    rotateSpeed = controlledRotationMm(frontBackDiff, -0.2);
                }
            }
            else
            {
                rotateSpeed = 0.0;
            }

            // Drive in the X direction should be the same as driving left/right
            robot.drive(speed, 0.0, rotateSpeed, 0.0);
            updateTelemetry(telemetry);

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