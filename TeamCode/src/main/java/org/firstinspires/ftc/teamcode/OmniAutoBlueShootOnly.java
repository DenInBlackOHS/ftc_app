package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoBlueShootOnly", group ="Auto")

public class OmniAutoBlueShootOnly extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 90.0;
        setupRobotParameters(4, 40);
//        setupVuforiaImages();
//        setupTextToSpeech();

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        telemetry.addLine("Set");
        updateTelemetry(telemetry);

        // Move to shooting position from wall using range sensor.
        telemetry.addLine("Move To Wall");
        updateTelemetry(telemetry);

        driveToWall(1.0, 0.2, 30.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.MID_LOW_SHOOT_SPEED);
        rotateRobot(0.4, 90.0, 7000);
        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }
        shoot(2000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // We should be able to acquire the gears target here
//        rotateRobotToAngle(0.6, ROBOT_ANGLE - 9.0, 7000);
//        if(isStopRequested())
//        {
//            return;
//        }
        // Move towards the wall a distance we should pick up the line and beacon colors
        driveToWall(1.0, 0.2, 5.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }
        rotateRobotToAngle(0.6, ROBOT_ANGLE, 7000);
        if(isStopRequested())
        {
            return;
        }

        if(isStopRequested())
        {
            return;
        }
        acquireLineOds(30000, ROBOT_ANGLE, true);
        if(isStopRequested())
        {
            return;
        }
        captureBlueBeacon(30000);
        if(isStopRequested())
        {
            return;
        }
        // Try getting further from the wall see if we can make the run to beacon 2.
        driveToWall(1.0, 0.2, 10.0, 5000, false);
        driveDistanceForwardOnHeading(1.0, 55.0, 3000, true);
        if(isStopRequested())
        {
            return;
        }

        acquireLineOds(30000, ROBOT_ANGLE, true);
        if(isStopRequested())
        {
            return;
        }
        captureBlueBeacon(30000);

        while(!isStopRequested())
        {
            driveAtHeading(1.0, 0.2, ROBOT_ANGLE + 50.0, ROBOT_ANGLE);
        }
//        enableVuforiaTracking();
//        acquireRedTarget1(30000);
//        if(isStopRequested())
//        {
//            // +14 degrees
//            // -880 mm
//            return;
//        }
        endAuto();
    }
}
