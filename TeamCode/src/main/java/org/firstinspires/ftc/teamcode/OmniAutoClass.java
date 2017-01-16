package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Created by Ethan on 10/23/2016.
 */
public abstract class OmniAutoClass extends LinearOpMode {
    // Vuforia Code
    public static final String TAG = "Vuforia Omni Autonomous";
    private OpenGLMatrix lastLocation = null;
    private OpenGLMatrix lastTargetLocation = null;
    private TextToSpeech textToSpeech = null;
    private String targetObtained = "";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaTrackables allTargets = null;
    // Gears
    VuforiaTrackable redTarget1 = null;
    // Tools
    VuforiaTrackable redTarget2 = null;
    // Wheels
    VuforiaTrackable blueTarget1  = null;
    // Legos
    VuforiaTrackable blueTarget2  = null;

    public static float mmPerInch = OmniAutoClass.MM_PER_INCH;
    public static float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    public static float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    public static int GEARS_NUMBER = 3;
    public static int TOOLS_NUMBER = 1;
    public static int LEGOS_NUMBER = 2;
    public static int WHEELS_NUMBER = 0;
    // End Vuforia Code

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
     * Enable text to speech for target recognition
     */
    public void setupTextToSpeech()
    {
        textToSpeech = new TextToSpeech(
                hardwareMap.appContext,
                new TextToSpeech.OnInitListener()
                {
                    @Override
                    public void onInit(int status)
                    {
                        if (status != TextToSpeech.ERROR)
                        {
                            textToSpeech.setLanguage(Locale.US);
                        }
                    }
                });
    }

    /**
     * Setup the vuforia trackables
     */
    public void setupVuforiaImages()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATaHrPr/////AAAAGYhG118G0EZgjFy6T7Snt3otqlgNSultuXDM66X1x1QK3ov5GUJcqL/9RTkdWkDlZDRxBKTAWm/szD7VmJteuQd2WfAk1t8qraapAsr2b4H5k5r4IpIO0UZghwNqhUqfZnCYl3e9tmmuocgZlfLXt4Xw+IAGxZ5e9MaQLR5lTv9/aFO1/CnH9/8jvnSq5NGeLrCHA6BtvqS30sAv7NYX8gz79MHaNiGZvyrUXZslbp2HHkehCocBbc080NrnYCouuUCqIbaMFl4ei8/ViSvdvtJDks4ox5KynBth4HaLHYpYkK3T2XJ1dBab6KfrWn6dm8ug7tfHTy68wLqWev7IWB0oPcqGOY+bZiz343VteHzk";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        allTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        redTarget1 = allTargets.get(GEARS_NUMBER);
        redTarget1.setName("Gears"); // Gears
        redTarget2 = allTargets.get(TOOLS_NUMBER);
        redTarget2.setName("Tools");  // Tools

        blueTarget1  = allTargets.get(WHEELS_NUMBER);
        blueTarget1.setName("Wheels");  // Wheels
        blueTarget2  = allTargets.get(LEGOS_NUMBER);
        blueTarget2.setName("Legos");  // Legos

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(allTargets);

       /*
        * To place the Gears Target on the Red Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Then we rotate it  90 around the field's Z access to face it away from the audience.
        * - Finally, we translate it back along the X axis towards the red audience wall.
        */
        // Gears
        OpenGLMatrix redTarget1LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -12*mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget1.setLocation(redTarget1LocationOnField);

        // Tools
        OpenGLMatrix redTarget2LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 36*mmPerInch, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget2.setLocation(redTarget2LocationOnField);

       /*
        * To place the Wheels Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        // Wheels
        OpenGLMatrix blueTarget1LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(12*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget1.setLocation(blueTarget1LocationOnField);

        // Legos
        OpenGLMatrix blueTarget2LocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-36*mmPerInch, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget2.setLocation(blueTarget2LocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(-mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 90, 0, 0));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    }

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

    /**
     *
     * @param position - The current encoder position
     * @param destination - The desired encoder position
     * @param speed - The speed of travel used to get direction
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean reachedClickPosition(int position, int destination, double speed, boolean reverseEncoders)
    {
        boolean result = false;

        if(reverseEncoders)
        {
            if (speed < 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        }
        else {
            if (speed > 0) {
                if (position >= destination) {
                    result = true;
                }
            } else {
                if (position <= destination) {
                    result = true;
                }
            }
        }
        return result;
    }

    /**
     *
     * @param speed - The driving power
     * @param rotateSpeed - The rotational speed to correct heading errors
     * @param driveAngle - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle)
    {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        double gyroReading = robot.readGyro();
        deltaAngle = deltaAngle(headingAngle, gyroReading);

        if(Math.abs(deltaAngle) > SAME_ANGLE)
        {
            if(deltaAngle > 0.0)
            {
                rotateSpeed = -rotateSpeed;
            }
        }
        else
        {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.cos(Math.toRadians(driveAngle));
        yPower = speed * Math.sin(Math.toRadians(driveAngle));
        telemetry.addData("Speed: ", speed);
        telemetry.addData("Drive xPower: ", xPower);
        telemetry.addData("Drive yPower: ", yPower);
        telemetry.addData("Rotate Speed: ", rotateSpeed);
        telemetry.addData("Gyro Reading: ", gyroReading);
        telemetry.addData("Heading Angle: ", headingAngle);
        telemetry.addData("Delta Angle: ", deltaAngle);
        telemetry.addData("Drive Angle: ", driveAngle);
        updateTelemetry(telemetry);
        robot.drive(xPower, yPower, rotateSpeed, 0.0);
    }

    /**
     *
     * @param speed - The driving power
     * @param rotateSpeed - The rotational speed to correct heading errors
     * @param driveAngle - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeadingForTime(double speed, double rotateSpeed, double driveAngle, double headingAngle, int driveTime)
    {
        int sleepTime = 0;
        final int DELTA_SLEEP = 50;

        while(!isStopRequested() && sleepTime < driveTime)
        {
            driveAtHeading(speed, rotateSpeed, driveAngle, headingAngle);
            sleep(50);
            sleepTime += DELTA_SLEEP;
        }
        robot.setAllDriveZero();
    }

    /**
     *
     * @param speed - The maximum speed for the robot
     * @param rotateSpeed - The maximum rotational speed for the robot
     * @param driveAngle - The angle to drive at
     * @param headingAngle - The angle to maintain heading for
     * @param distance - The distance to travel
     * @param maxTime - The time to wait before giving up
     */
    private void driveDistanceAtAngleOnHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle, double distance, int maxTime, boolean reverseEncoders)
    {
        //temporary fix until we understand
        double correctedDistance = distance / 1.75;

        int sleepTime = 0;
        final int deltaSleep = 50;
        int position = robot.leftMotorFore.getCurrentPosition();
        int finalEncoderValue;
        double gyroReading = robot.readGyro();

        if(reverseEncoders) {
            if (speed < 0) {
                finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
            } else {
                finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
            }
        }
        else {
            if (speed < 0) {
                finalEncoderValue = position - (int) (correctedDistance * clicksPerInch);
            } else {
                finalEncoderValue = position + (int) (correctedDistance * clicksPerInch);
            }
        }

        while ((!reachedClickPosition(position, finalEncoderValue, speed, reverseEncoders) && (sleepTime < maxTime)))
        {
            String myTelemetry = "Current Encoder: " + position + " Destination Encoder: " + finalEncoderValue;
            telemetry.addLine(myTelemetry);
            telemetry.addData("Setting Power: ", speed);
            telemetry.addData("Sleep Time: ", sleepTime);

            // Since this is a driveForward function, the Y Axis is the only important axis
            driveAtHeading(speed, rotateSpeed, driveAngle, gyroReading);
            updateTelemetry(telemetry);

            sleep(deltaSleep);
            sleepTime += deltaSleep;
            position = robot.leftMotorFore.getCurrentPosition();
            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }

        robot.setAllDriveZero();
    }

    /**
     *
     * @param speed - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime - The time allowed before exiting without completing
     */
    public void driveDistanceForwardOnHeading(double speed, double distance, int maxTime, boolean reverseEncoders)
    {
        driveDistanceAtAngleOnHeading(speed, 0.1, 0.0, robot.readGyro(), distance, maxTime, reverseEncoders);
    }

    /**
     *
     * @param speed - Max speed to run robot
     * @param distance - Desired distance to travel in inches
     * @param maxTime - The time allowed before exiting without completing
     */
    public void driveDistanceSidewaysOnHeading(double speed, double distance, int maxTime, boolean reverseEncoders)
    {
        driveDistanceAtAngleOnHeading(speed, 0.1, 90.0, robot.readGyro(), distance, maxTime, reverseEncoders);
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
            telemetry.addData("Current Angle: ", gyroReading);
            telemetry.addData("Destination Angle: ", targetAngle);
            telemetry.addData("Sleep Time: ", sleepTime);
            telemetry.addData("Delta Angle: ", angleRemaining);

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

        robot.setAllDriveZero();
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
        robot.setLiftMotorPower(HardwareOmnibot.LIFT_SPEED);
        robot.setSweeperMotorPower(HardwareOmnibot.SWEEP_SPEED);

        updateTelemetry(telemetry);

        // Let things happen, time passed in is total time, not just the time
        // to lift and fire the balls.
        sleep(fireTime);

        robot.setLiftMotorPower(0.0);
        robot.setSweeperMotorPower(0.0);
    }

    /**
     *
     * @param distanceToTravelMm - How far we are traveling in mm
     * @param maxSpeed - Top speed to travel
     * @return The speed to go with the distance remaining
     */
    private double controlledDeceleration(double distanceToTravelMm, double maxSpeed)
    {
        final double superFastDistance = 400.0;
        final double fastDistance = 200.0;
        final double mediumDistance = 100.0;
        final double fastDivider = 1.4;
        final double mediumDivider = 2.0;
        final double slowDivider = 3.0;

        double result = 0.0;

        if(distanceToTravelMm > superFastDistance)
        {
            result = maxSpeed;
        }
        else if(distanceToTravelMm > fastDistance)
        {
            result = maxSpeed / fastDivider;
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
        final double fastAngle = 45.0;
        final double mediumAngle = 15.0;
        final double mediumDivider = 2.0;
        final double slowDivider = 4.0;
        double angleToTravelAbs = Math.abs(angleToTravel);

        double result = 0.0;

        if(angleToTravelAbs > fastAngle)
        {
            result = maxSpeed;
        }
        else if(angleToTravelAbs > mediumAngle)
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
     * @param distanceFromWall - The distance to make the robot parallel to the wall in inches
     * @param timeout - The maximum amount of time to wait until giving up
     */
    public void driveToWall(double maxSpeed, double maxRotation, double distanceFromWall, int timeout, boolean frontSensor)
    {
        double allowedDistanceError = 10;
        int sleepTime = 0;
        final int deltaSleep = 50;
        double maxSpeedAbs = Math.abs(maxSpeed);
        double sensorDistance = 0.0;
        double sensorDelta = 0.0;
        double sensorError = 0.0;
        double speed = 0.0;
        double distanceFromWallMm = distanceFromWall * mmPerInch;
        double gyroAngle = robot.readGyro();
        double driveAngle = gyroAngle;
        double robotRotation = maxRotation;

//        if(frontSensor)
//        {
//            sensorDistance = robot.readFrontRangeSensor();
//            driveAngle -= 90.0;
//            if(gyroAngle < 0.0)
//            {
//                gyroAngle += 360.0;
//            }
//            else if(gyroAngle > 360.0)
//            {
//                gyroAngle -= 360.0;
//            }
//        }
//        else
//        {
            sensorDistance = robot.readBackRangeSensor();
//        }
        sensorDelta = sensorDistance - distanceFromWallMm;
        sensorError = Math.abs(sensorDelta);

        while(((sensorError > allowedDistanceError)) && (sleepTime < timeout))
        {
            // Calculate the linear driving
            if(sensorDistance < distanceFromWallMm)
            {
                // Drive away from wall
                speed = controlledDeceleration(sensorError, maxSpeedAbs);
            } else if (sensorDistance > distanceFromWallMm)
            {
                // Drive towards wall
                speed = controlledDeceleration(sensorError, -maxSpeedAbs);
            }
            else
            {
                // We just need to rotate
                speed = 0.0;
            }

            telemetry.addData("Target Distance: ", distanceFromWallMm);
            telemetry.addData("Sensor Distance: ", sensorDistance);
            telemetry.addData("Delta Distance: ", sensorDelta);
            telemetry.addData("Max Power: ", maxSpeedAbs);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Gyro Angle: ", gyroAngle);
            telemetry.addData("Sleep Time: ", sleepTime);
            // Drive in the X direction should be the same as driving left/right
            driveAtHeading(speed, maxRotation, driveAngle, gyroAngle);
            updateTelemetry(telemetry);

            // Let things happen
            sleep(deltaSleep);
            sleepTime += deltaSleep;

            // Get new readings
//            if(frontSensor)
//            {
//                sensorDistance = robot.readFrontRangeSensor();
//            }
//            else
//            {
                sensorDistance = robot.readBackRangeSensor();
//            }
            sensorDelta = sensorDistance - distanceFromWallMm;
            sensorError = Math.abs(sensorDelta);

            if (isStopRequested()) {
                // If stop has been requested, break out of the while loop.
                break;
            }
        }
        robot.setAllDriveZero();
    }

    public void endAuto() {
        robot.disableRangeSensors();
        robot.disableColorSensors();
        while(!isStopRequested())
        {
            sleep(100);
        }
    }

    public void enableVuforiaTracking()
    {
        /** Start tracking the data sets we care about. */
        allTargets.activate();
    }

    public void acquireLineOds(int maxTime, double heading, double driveAngle, boolean farSide)
    {
        boolean targetAcquired = false;
        boolean lineAcquired = false;
        double lineReading = 0.0;
        int sleepTime = 0;
        final int deltaSleep = 10;

        while (opModeIsActive() && !targetAcquired && (sleepTime < maxTime))
        {
            if(!lineAcquired)
            {
                lineReading = robot.readOds();
                telemetry.addData("LA ODS", lineReading);
                if(lineReading >= HardwareOmnibot.MIN_ODS_VALUE)
                {
                    lineAcquired = true;
                    if(!farSide)
                    {
                        robot.setAllDriveZero();
                        targetAcquired = true;
                    }
                }
                else
                {
                    driveAtHeading(0.15, 0.1, driveAngle, heading);
                    sleep(deltaSleep);
                }
            }
            else if(!targetAcquired)
            {
                lineReading = robot.readOds();
                telemetry.addData("LA ODS", lineReading);
                if(lineReading <= HardwareOmnibot.MIN_ODS_VALUE)
                {
                    robot.setAllDriveZero();
                    targetAcquired = true;
                }
                else
                {
                    driveAtHeading(0.15, 0.1, driveAngle, heading);
                    sleep(deltaSleep);
                }
            }
            telemetry.addData("ODS Read: ", lineReading);
            telemetry.addData("Heading: ", heading);
            telemetry.addData("Color Sensor Blue Front: ", robot.readBlueFrontColorSensor());
            telemetry.addData("Color Sensor Red Front: ", robot.readRedFrontColorSensor());
            telemetry.addData("Color Sensor Blue Back: ", robot.readBlueBackColorSensor());
            telemetry.addData("Color Sensor Red Back: ", robot.readBlueBackColorSensor());
            telemetry.update();
        }
    }

    private long pressCorrectButtonRed(double heading)
    {
        final double FRONT_BUTTON_DELTA = 8.0;
        final double BACK_BUTTON_DELTA = 8.0;
        final double READING_DISTANCE = 5.5;
        long startTime = 0;

        // We are looking for blue because it is a stronger returning color
        if(robot.readBlueFrontColorSensor() > HardwareOmnibot.MIN_BLUE_COLOR_VALUE)
        {
            // Angle the robot towards the back button
            rotateRobotToAngle(0.4, heading - BACK_BUTTON_DELTA, 3000);
            if(isStopRequested())
            {
                return 0;
            }
            // The sensor is in the back, so closer to the wall when we press
            driveToWall(0.4, 0.1, 1.0, 1500, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Start the timer in case we pressed the wrong button
            startTime = System.currentTimeMillis();
            // Drive away from the wall to read the color sensors
            driveToWall(0.4, 0.1, READING_DISTANCE, 3000, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Straighten out the robot
            rotateRobotToAngle(0.4, heading - 2, 3000);
        }
        else
        {
            // Rotate the robot towards the front button.
            rotateRobotToAngle(0.4, heading + FRONT_BUTTON_DELTA, 3000);
            if(isStopRequested())
            {
                return 0;
            }
            // This will probably time out because the sensor is away from the wall because of the angle
            driveToWall(0.4, 0.1, 2.0, 1500, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Start the timer in case we pressed the wrong button
            startTime = System.currentTimeMillis();
            // Drive away from the wall to read the color sensors
            driveToWall(0.4, 0.1, READING_DISTANCE, 3000, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Straighten out the robot
            rotateRobotToAngle(0.4, heading - 2, 3000);
        }

        return startTime;
    }

    public void captureRedBeacon(int maxTime, double heading)
    {
        final double READING_DISTANCE = 5.5;
        long startTime = 0;

        // get close to beacon to read color
        rotateRobotToAngle(0.4, heading, 3000);
        if(isStopRequested())
        {
            return;
        }
        driveToWall(0.4, 0.1, READING_DISTANCE, 2000, false);
        if(isStopRequested())
        {
            return;
        }

        startTime = pressCorrectButtonRed(heading);
        if(isStopRequested())
        {
            return;
        }

        // If one side is blue, we pressed the wrong button, or neither button.
        if((robot.readBlueFrontColorSensor() > HardwareOmnibot.MIN_BLUE_COLOR_VALUE) ||
                (robot.readBlueBackColorSensor() > HardwareOmnibot.MIN_BLUE_COLOR_VALUE))

        {
            // We pressed the wrong button, so have to wait 5 seconds.
            if((robot.readBlueBackColorSensor() > HardwareOmnibot.MIN_BLUE_COLOR_VALUE) &&
                    (robot.readBlueFrontColorSensor() > HardwareOmnibot.MIN_BLUE_COLOR_VALUE))
            {
                sleep(4750);
                // We can press either button, so just go straight in.
                driveToWall(0.4, 0.1, 1.0, 1500, false);
                if(isStopRequested())
                {
                    return;
                }
                // Pull away from the wall
                driveToWall(0.4, 0.1, READING_DISTANCE, 3000, false);
                rotateRobotToAngle(0.4, heading, 3000);
            }
            else
            {
                pressCorrectButtonRed(heading);
            }
        }
    }

    private long pressCorrectButtonBlue(double heading)
    {
        final double FRONT_BUTTON_DELTA = 8.0;
        final double BACK_BUTTON_DELTA = 8.0;
        long startTime = 0;

        // We are looking for blue because it is a stronger returning color
        // Front is the back button for blue.
        if(robot.readBlueFrontColorSensor() > HardwareOmnibot.MIN_BLUE_COLOR_VALUE)
        {
            // Angle the robot towards the front button
            rotateRobotToAngle(0.4, heading + BACK_BUTTON_DELTA, 3000);
            if(isStopRequested())
            {
                return 0;
            }
            // The sensor is in the "back", so further from the wall when we press
            driveToWall(0.4, 0.1, 0.0, 1500, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Start the timer in case we pressed the wrong button
            startTime = System.currentTimeMillis();
            // Drive away from the wall to read the color sensors
            driveToWall(0.4, 0.1, 5.0, 3000, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Straighten out the robot
            rotateRobotToAngle(0.4, heading + 2, 3000);
        }
        else
        {
            // Rotate the robot towards the front button.
            rotateRobotToAngle(0.4, heading - FRONT_BUTTON_DELTA, 3000);
            if(isStopRequested()) {
                return 0;
            }

            driveToWall(0.4, 0.1, 0.0, 1500, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Start the timer in case we pressed the wrong button
            startTime = System.currentTimeMillis();
            // Drive away from the wall to read the color sensors
            driveToWall(0.4, 0.1, 5.0, 3000, false);
            if(isStopRequested())
            {
                return 0;
            }
            // Straighten out the robot
            rotateRobotToAngle(0.4, heading + 2, 3000);
        }

        return startTime;
    }

    public void captureBlueBeacon(int maxTime, double heading)
    {
        final double READING_DISTANCE = 5.5;
        long startTime = 0;

        // get close to beacon to read color
        rotateRobotToAngle(0.4, heading, 3000);
        if(isStopRequested())
        {
            return;
        }
        driveToWall(0.4, 0.1, READING_DISTANCE, 2000, false);
        if(isStopRequested())
        {
            return;
        }

        startTime = pressCorrectButtonBlue(heading);
        if(isStopRequested())
        {
            return;
        }

        // If one side is red, we pressed the wrong button, or neither button.
        if((robot.readBlueFrontColorSensor() < HardwareOmnibot.MIN_BLUE_COLOR_VALUE) ||
                (robot.readBlueBackColorSensor() < HardwareOmnibot.MIN_BLUE_COLOR_VALUE))

        {
            // We pressed the wrong button, so have to wait 5 seconds.
            if((robot.readBlueBackColorSensor() < HardwareOmnibot.MIN_BLUE_COLOR_VALUE) &&
                    (robot.readBlueFrontColorSensor() < HardwareOmnibot.MIN_BLUE_COLOR_VALUE))
            {
                sleep(4750);
                // We can press either button, so just go straight in.
                driveToWall(0.4, 0.1, 1.0, 1500, false);
                if(isStopRequested())
                {
                    return;
                }
                // Pull away from the wall
                driveToWall(0.4, 0.1, READING_DISTANCE, 3000, false);
                rotateRobotToAngle(0.4, heading, 3000);
            }
            else
            {
                pressCorrectButtonBlue(heading);
            }
        }
    }

    public void acquireRedTarget1(int maxTime)
    {
        boolean closingYGap = true;
        boolean closingXGap = true;
        double xDistance = 0.0;
        double yDistance = 0.0;
        double targetAngle = 0.0;
        double targetDistance = 0.0;
        double destinationDistance = 0.0;
        double destinationAngle = 0.0;
        boolean reachedDestination = false;
        long startTime = System.currentTimeMillis();
        long elapsedTime = 0;

        while (opModeIsActive() && !reachedDestination && (elapsedTime < maxTime))
        {
            for (VuforiaTrackable trackable : allTrackables)
            {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix targetLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null)
                {
                    lastLocation = robotLocationTransform;
                    if((textToSpeech != null) &&(!targetObtained.equals(trackable.getName()))) {
                        String sentence = "";
                        if(!targetObtained.isEmpty()) {
                            sentence = "We have lost objective " + targetObtained + ".  ";
                        }
                        targetObtained = trackable.getName();
                        sentence += "Obtained objective " + targetObtained;
                        textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                    }
                }
                if (targetLocationTransform != null)
                {
                    lastTargetLocation = targetLocationTransform;
                    VectorF translation = lastTargetLocation.getTranslation();
                    Orientation orientation = Orientation.getOrientation(lastTargetLocation, AxesReference.EXTRINSIC,
                            AxesOrder.XYZ, AngleUnit.DEGREES);

                    targetAngle = orientation.secondAngle;
                    targetDistance = translation.get(2);
                    // This will get us 15 mm from the target so we can do color analysis
                    destinationDistance = targetDistance + 90;
                    destinationAngle = 90.0 + targetAngle;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastTargetLocation != null)
            {
//                telemetry.addData("TargPos", format(lastTargetLocation));
//                telemetry.addData("Target Angle: ", targetAngle);
//                telemetry.addData("Target Distance: ", targetDistance);
//                telemetry.addData("Destination Angle: ", destinationAngle);
//                telemetry.addData("Destination Distance: ", destinationDistance);
                double speed = controlledDeceleration(destinationDistance, 0.7);
                if(Math.abs(destinationDistance) > 10.0)
                {
                    if(closingYGap)
                    {
                        yDistance = destinationDistance * Math.cos(Math.toRadians(destinationAngle));
                        telemetry.addData("Y Distance: ", yDistance);
                        telemetry.addData("Dest Angle: ", destinationAngle);
//                telemetry.addData("Target Distance: ", targetDistance);
                        if(Math.abs(yDistance) < 1.0)
                        {
                            closingYGap = false;
                            robot.setAllDriveZero();
                        }
                        else
                        {
                            speed = controlledDeceleration(yDistance, 0.5);
                            driveAtHeading(speed, 0.1, 0.0, 270.0);
                        }
                    }
                    else
                    {
                        xDistance = destinationDistance * Math.cos(Math.toRadians(destinationAngle));
                        if(Math.abs(xDistance) < 1.0)
                        {
                            closingXGap = false;
                            robot.setAllDriveZero();
                        }
                        else
                        {
                            speed = controlledDeceleration(xDistance, 0.5);
                            driveAtHeading(speed, 0.1, 90.0, 270.0);
                        }
                    }
                }
                else
                {
                    robot.setAllDriveZero();
                    reachedDestination = true;
                }
            }
            else
            {
                telemetry.addData("TargPos", "Unknown");
                telemetry.addData("Target Angle: ", "Unknown");
                telemetry.addData("Target Distance: ", "Unknown");
            }
            elapsedTime = System.currentTimeMillis() - startTime;
            telemetry.addData("Elapsed Time: ", elapsedTime);
            telemetry.update();
        }
    }
    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}