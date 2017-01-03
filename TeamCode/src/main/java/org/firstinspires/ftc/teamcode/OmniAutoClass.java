package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ethan on 10/23/2016.
 */
public abstract class OmniAutoClass extends LinearOpMode {

    HardwareOmnibot robot = new HardwareOmnibot();

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

    /**
     *
     * @param newWheelSize - The size of the wheels, used to calculate encoder clicks per inch
     * @param newMotorRatio - The motor gearbox ratio, used to calculate encoder clicks per inch
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

    /**
     *
     * @param position - The current encoder position
     * @param destination - The desired encoder position
     * @param speed - The speed of travel used to get direction
     * @return - Boolean true we have reached destination, false we have not
     */
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

    /**
     *
     * @param speed - The driving power
     * @param driveAngle - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double driveAngle, double headingAngle)
    {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        final double ROTATE_SPEED = 0.1;
        double rotateSpeed = 0.0;
        double gyroReading = robot.readGyro();
        deltaAngle = deltaAngle(headingAngle, gyroReading);

        if(Math.abs(deltaAngle) > SAME_ANGLE)
        {
            if(deltaAngle > 0.0)
            {
                rotateSpeed = -ROTATE_SPEED;
            }
            else
            {
                rotateSpeed = ROTATE_SPEED;
            }
        }
        else
        {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.sin(Math.toRadians(driveAngle));
        yPower = speed * Math.cos(Math.toRadians(driveAngle));
        telemetry.addData("Speed: ", speed);
        telemetry.addData("Drive xPower: ", xPower);
        telemetry.addData("Drive yPower: ", yPower);
        telemetry.addData("Rotate Speed: ", rotateSpeed);
        telemetry.addData("Gyro Reading: ", gyroReading);
        telemetry.addData("Delta Angle: ", deltaAngle);
        telemetry.addData("Drive Angle: ", driveAngle);
        updateTelemetry(telemetry);
        robot.drive(xPower, yPower, rotateSpeed, 0.0);
    }

    /**
     *
     * @param speed - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime - The time allowed before exiting without completing
     */
    public void driveForwardOnHeading(double speed, double distance, int maxTime)
    {
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

        while ((!reachedClickPosition(position, finalEncoderValue, speed) && (sleepTime < maxTime)))
        {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + finalEncoderValue;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Since this is a driveForward function, the Y Axis is the only important axis
            driveAtHeading(speed, 0.0, gyroReading);
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

    /**
     *
     * @param destinationAngle - The target angle to reach, between 0.0 and 360.0
     * @param gyroReading - The current angle of the robot
     * @return The minumum angle to travel to get to the destination angle
     */
    private double deltaAngle(double destinationAngle, double gyroReading)
    {
        double result = 0.0;
        double leftResult = 0.0;
        double rightResult = 0.0;

        if(gyroReading > destinationAngle)
        {
            leftResult = gyroReading - destinationAngle;
            rightResult = 360.0 - gyroReading + destinationAngle;
        }
        else
        {
            leftResult = gyroReading + 360.0 - destinationAngle;
            rightResult = destinationAngle - gyroReading;
        }

        if(leftResult < rightResult)
        {
            result = -leftResult;
        }
        else
        {
            result = rightResult;
        }

        return result;
    }

    /**
     *
     * @param speed - The maximum speed to rotate the robot
     * @param targetAngle - The 0-360 degree angle to rotate the robot to
     * @param maxTime - The time to allow before quiting
     */
    public void rotateRobotToAngle(double speed, double targetAngle, int maxTime) {
        double gyroReading = robot.readGyro();
        int sleepTime = 0;
        final int deltaSleep = 50;
        double angleRemaining = 0.0;
        final double SAME_ANGLE = 1.0;
        double rotateSpeed = 0.0;

        angleRemaining = deltaAngle(targetAngle, gyroReading);
        while (Math.abs(angleRemaining) > SAME_ANGLE && (sleepTime < maxTime)) {
            String myTelemetry = "Current Angle: " + gyroReading + " Destination Angle: " + targetAngle;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Sleep Time: ", sleepTime);

            if(angleRemaining > 0.0)
            {
                // Positive angle, need to rotate right
                rotateSpeed = - controlledRotationAngle(angleRemaining, speed);
            }
            else
            {
                // Negative angle, need to rotate left;
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

    /**
     *
     * @param speed - Max speed to rotate
     * @param angle - Angle in degrees to rotate.  Positive right, negative left
     * @param maxTime - Timeout to quit trying
     */
    public void rotateRobot(double speed, double angle, int maxTime) {
        double gyroReading = robot.readGyro();
        double targetAngle = gyroReading + angle;

        // We won't do circles with this function, just minimum rotation.
        // Get the destination gyro angle
        while(targetAngle > 360.0)
        {
            targetAngle -= 360.0;
        }
        while(targetAngle < 0.0)
        {
            targetAngle += 360.0;
        }

        rotateRobotToAngle(speed, targetAngle, maxTime);
    }

    /**
     *
     * @param fireTime - The time to allow the particles to be shot before stopping
     */
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
        final double fastAngle = 30.0;
        final double mediumAngle = 15.0;
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
        double backDistance = 0.0;
        double backDelta = 0.0;
        double backError = 0.0;
        double speed = 0.0;

        backDistance = robot.readBackRangeSensor();
        backDelta = backDistance - distanceFromWallMm;
        backError = Math.abs(backDelta);

        while(((backError > allowedDistanceError)) && (sleepTime < timeout))
        {
            String myTelemetry = " Back Distance: " + backDistance;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Max Power: ", maxSpeedAbs);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Calculate the linear driving
            if(backDistance < distanceFromWallMm)
            {
                // Drive away from wall
                speed = controlledDeceleration(backError, maxSpeedAbs);
            } else if (backDistance > distanceFromWallMm)
            {
                // Drive towards wall
                speed = controlledDeceleration(backError, -maxSpeedAbs);
            }
            else
            {
                // We just need to rotate
                speed = 0.0;
            }

            // Drive in the X direction should be the same as driving left/right
            driveAtHeading(speed, 90.0, 0.0);
            updateTelemetry(telemetry);

            // Let things happen
            sleep(deltaSleep);
            sleepTime += deltaSleep;

            // Get new readings
            backDistance = robot.readBackRangeSensor();
            backDelta = backDistance - distanceFromWallMm;
            backError = Math.abs(backDelta);

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