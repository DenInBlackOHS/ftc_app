package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoBlue", group ="Auto")

public class OmniAutoBlue extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 90.0;
        final double DRIVE_ANGLE = 0.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        telemetry.addLine("Set");
        updateTelemetry(telemetry);

        // Move to shooting position from wall using range sensor.
        telemetry.addLine("Move To Wall");
        updateTelemetry(telemetry);

        driveToWall(1.0, 0.2, 24.0, 10000, true);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobotToAngle(1.0, ROBOT_ANGLE + 7.0, 7000);
        shoot(2000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // Move towards the wall a distance we should pick up the line and beacon colors
        driveToWall(1.0, 0.2, 5.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }
        rotateRobotToAngle(1.0, ROBOT_ANGLE, 7000);
        if(isStopRequested())
        {
            return;
        }

        acquireLineOds(30000, ROBOT_ANGLE, DRIVE_ANGLE, true);
        if(isStopRequested())
        {
            return;
        }
        captureBlueBeacon(30000, ROBOT_ANGLE);
        if(isStopRequested())
        {
            return;
        }
        // Try getting further from the wall see if we can make the run to beacon 2.
        driveToWall(1.0, 0.2, 10.0, 5000, false);
        if(isStopRequested())
        {
            return;
        }
        driveDistanceForwardOnHeading(1.0, 55.0, 3000, true);
        if(isStopRequested())
        {
            return;
        }

        acquireLineOds(30000, ROBOT_ANGLE, DRIVE_ANGLE, true);
        if(isStopRequested())
        {
            return;
        }
        captureBlueBeacon(30000, ROBOT_ANGLE);

        // This should knock the cap ball off, and set the robot on the pedestal
        driveAtHeadingForTime(1.0, 0.2, ROBOT_ANGLE + 37.0, ROBOT_ANGLE, 2500);
        if(isStopRequested())
        {
            return;
        }

        endAuto();
    }
}
