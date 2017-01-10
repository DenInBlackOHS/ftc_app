package org.firstinspires.ftc.teamcode;
import android.speech.tts.TextToSpeech;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
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

/**
 * Created by Ethan on 10/30/2016.
 */

@TeleOp(name="Omni: Auto Vuforia", group ="TeleOp")
public class OmniAutoVuforia extends OmniAutoClass {
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

    @Override
    public void runOpMode() throws InterruptedException{
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
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATaHrPr/////AAAAGYhG118G0EZgjFy6T7Snt3otqlgNSultuXDM66X1x1QK3ov5GUJcqL/9RTkdWkDlZDRxBKTAWm/szD7VmJteuQd2WfAk1t8qraapAsr2b4H5k5r4IpIO0UZghwNqhUqfZnCYl3e9tmmuocgZlfLXt4Xw+IAGxZ5e9MaQLR5lTv9/aFO1/CnH9/8jvnSq5NGeLrCHA6BtvqS30sAv7NYX8gz79MHaNiGZvyrUXZslbp2HHkehCocBbc080NrnYCouuUCqIbaMFl4ei8/ViSvdvtJDks4ox5KynBth4HaLHYpYkK3T2XJ1dBab6KfrWn6dm8ug7tfHTy68wLqWev7IWB0oPcqGOY+bZiz343VteHzk";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        double targetAngle = 0.0;
        double targetDistance = 0.0;
        VuforiaTrackables allTargets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable redTarget1 = allTargets.get(GEARS_NUMBER);
        redTarget1.setName("Gears"); // Gears
        VuforiaTrackable redTarget2 = allTargets.get(TOOLS_NUMBER);
        redTarget2.setName("Tools");  // Tools

        VuforiaTrackable blueTarget1  = allTargets.get(WHEELS_NUMBER);
        blueTarget1.setName("Wheels");  // Wheels
        VuforiaTrackable blueTarget2  = allTargets.get(LEGOS_NUMBER);
        blueTarget2.setName("Legos");  // Legos

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(allTargets);

        float mmPerInch = OmniAutoClass.MM_PER_INCH;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

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
        RobotLog.ii(TAG, "Red Target1 Gears=%s", format(redTarget1LocationOnField));

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
        RobotLog.ii(TAG, "Red Target2 Tools=%s", format(redTarget2LocationOnField));

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
        RobotLog.ii(TAG, "Blue Target1 Wheels=%s", format(blueTarget1LocationOnField));

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
        RobotLog.ii(TAG, "Blue Target2 Legos=%s", format(blueTarget2LocationOnField));

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
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        // Does this denote the phone being in portrait mode?
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        waitForStart();

        /** Start tracking the data sets we care about. */
        allTargets.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix targetLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    if(!targetObtained.equals(trackable.getName())) {
                        String sentence = "";
                        if(!targetObtained.isEmpty()) {
                            sentence = "We have lost objective " + targetObtained + ".  ";
                        }
                        targetObtained = trackable.getName();
                        sentence += "Obtained objective " + targetObtained;
                        textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                    }
                }
                if (targetLocationTransform != null) {
                    lastTargetLocation = targetLocationTransform;

//                    public String formatAsTransform(AxesReference axesReference, AxesOrder axesOrder, AngleUnit unit)
//                    formatAsTransform(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    VectorF translation = lastTargetLocation.getTranslation();
                    Orientation orientation = Orientation.getOrientation(lastTargetLocation, AxesReference.EXTRINSIC,
                            AxesOrder.XYZ, AngleUnit.DEGREES);

                    targetAngle = orientation.secondAngle;
                    targetDistance = translation.get(2);
//                    double yDistance = translation.get(0);
//                    double xDistance = translation.get(1);
//                    targetAngle = Math.toDegrees(Math.atan2(yDistance, xDistance));
//                    targetDistance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
                telemetry.addData("TargPos", format(lastTargetLocation));
                telemetry.addData("Target Angle: ", targetAngle);
                telemetry.addData("Target Dist: ", targetDistance);
            } else {
                telemetry.addData("Pos", "Unknown");
                telemetry.addData("TargPos", "Unknown");
                telemetry.addData("Target Angle: ", "Unknown");
                telemetry.addData("Target Dist: ", "Unknown");
            }
            telemetry.update();
        }

        updateTelemetry(telemetry);

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
//        shoot(.35, 4000);

//        telemetry.addLine("Done Shooting");
//        updateTelemetry(telemetry);

        // Check to see if the program should exit
//        if(isStopRequested())
//        {
//            return;
//        }
//        sleep(6000);

        // Check to see if the program should exit
//        if(isStopRequested())
//        {
//            return;
//        }
//        driveForward(-0.4, 72, 3000);
//        telemetry.addLine("Go");
//        updateTelemetry(telemetry);
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
