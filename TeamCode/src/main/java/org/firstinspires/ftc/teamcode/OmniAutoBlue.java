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
        final double ROBOT_ANGLE = 0.0;
        final double DRIVE_ANGLE = 270.0;
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

        driveAtHeadingForTime(1.0, 0.4, DRIVE_ANGLE, 25.0, 500);
        if(isStopRequested())
        {
            return;
        }

        // Rotate the robot to the far beacon drive angle.
        rotateRobotToAngle(0.7, ROBOT_ANGLE + 25.0, 7000);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // Move towards the wall a distance we should pick up the line and beacon colors
        driveToWall(1.0, 0.2, 10.0, 15000, false);
        if(isStopRequested())
        {
            return;
        }
        rotateRobotToAngle(0.7, ROBOT_ANGLE, 7000);
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

        // Move towards the wall a distance we should pick up the line and beacon colors
        driveToWall(1.0, 0.2, 32.0, 15000, false);
        if(isStopRequested())
        {
            return;
        }
        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobotToAngle(0.7, ROBOT_ANGLE + 85.0, 7000);
        driveAtHeadingForTime(1.0, 0.4, DRIVE_ANGLE - 90.0, ROBOT_ANGLE + 85.0, 150);
        shoot(2000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }

        rotateRobotToAngle(0.7, ROBOT_ANGLE + 25.0, 7000);

        // Move towards the wall a distance we should pick up the line and beacon colors
        driveToWall(1.0, 0.2, 10.0, 15000, false);
        if(isStopRequested())
        {
            return;
        }
        rotateRobotToAngle(0.7, ROBOT_ANGLE, 7000);
        if(isStopRequested())
        {
            return;
        }

        driveDistanceSidewaysOnHeading(-1.0, 25.0, 3000, false);
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

        driveAtHeadingForTime(1.0, 0.2, DRIVE_ANGLE + 120.0, ROBOT_ANGLE, 2000);
        endAuto();
    }
}
