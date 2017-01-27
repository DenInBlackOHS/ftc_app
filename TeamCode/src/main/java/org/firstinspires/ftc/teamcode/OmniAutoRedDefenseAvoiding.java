package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Ethan on 10/30/2016.
 */

@Autonomous(name="Omni: AutoRedDefenseAvoiding", group ="Expiremental")

public class OmniAutoRedDefenseAvoiding extends OmniAutoClass {

    @Override
    public void runOpMode() throws InterruptedException
    {
        final double ROBOT_ANGLE = 269.0;
        final double DRIVE_ANGLE = 0.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        driveToWall(1.0, 0.2, 24.0, 10000, true);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobot(0.7, ROBOT_ANGLE - 180.0, 7000);
        shoot(2000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // We should be able to acquire the gears target here
        rotateRobotToAngle(1.0, ROBOT_ANGLE - 26.0, 7000);
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
        rotateRobotToAngle(1.0, ROBOT_ANGLE, 7000);
        if(isStopRequested())
        {
            return;
        }

        driveDistanceForwardOnHeading(1.0, 7.0, 3000, false);
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
        driveToWall(1.0, 0.2, 6.0, 5000, false);
        driveDistanceForwardOnHeading(1.0, 55.0, 3000, false);
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
        final double IMPACT_ANGLE = -40.0;
        rotateRobotToAngle(1.0, ROBOT_ANGLE - IMPACT_ANGLE, 7000);
        driveAtHeadingForTime(1.0, 0.2, ROBOT_ANGLE - 30.0, ROBOT_ANGLE - IMPACT_ANGLE, 3500);
        spinRobot(1.0, 60.0, 3000);

        endAuto();
    }
}
