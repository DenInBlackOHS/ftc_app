package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoRed", group ="Auto")

public class OmniAutoRed extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 179.0;
        final double DRIVE_ANGLE = -90.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        driveToWall(1.0, 0.2, 24.0, 5000, true);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        shoot(2000);
        rotateRobot(0.4, ROBOT_ANGLE - 7.0, 7000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // We should be able to acquire the gears target here
        rotateRobotToAngle(1.0, ROBOT_ANGLE - 22.0, 7000);
        if(isStopRequested())
        {
            return;
        }
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

        driveDistanceSidewaysOnHeading(-1.0, 3.0, 3000, true);
        if(isStopRequested())
        {
            return;
        }
        acquireLineOds(30000, ROBOT_ANGLE, DRIVE_ANGLE, false);
        if(isStopRequested())
        {
            return;
        }
        captureRedBeacon(30000, ROBOT_ANGLE);
        if(isStopRequested())
        {
            return;
        }
        driveDistanceSidewaysOnHeading(-1.0, 55.0, 3000, true);
        if(isStopRequested())
        {
            return;
        }

        acquireLineOds(30000, ROBOT_ANGLE, DRIVE_ANGLE, false);
        if(isStopRequested())
        {
            return;
        }
        captureRedBeacon(30000, ROBOT_ANGLE);
        if(isStopRequested())
        {
            return;
        }

        // This should knock the cap ball off, and set the robot on the pedestal
        driveAtHeadingForTime(1.0, 0.2, ROBOT_ANGLE - 50.0, ROBOT_ANGLE, 3500);

        endAuto();
    }
}
