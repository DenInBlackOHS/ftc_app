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
        final double ROBOT_ANGLE = 0.0;
        final double DRIVE_ANGLE = 90.0;
        final double AUTO_ANGLE = 325.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        robot.resetGyro();

        driveAtHeadingForTime(1.0, 0.4, DRIVE_ANGLE, AUTO_ANGLE, 500);
        if(isStopRequested())
        {
            return;
        }

        // Rotate the robot to the far beacon drive angle.
        rotateRobotToAngle(0.7, ROBOT_ANGLE + AUTO_ANGLE, 7000);

        driveToWall(1.0, 0.2, 8.0, 15000, false);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        rotateRobotToAngle(0.7, ROBOT_ANGLE, 7000);
        if(isStopRequested())
        {
            return;
        }
        driveDistanceSidewaysOnHeading(1.0, 10.0, 3000, false);
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

        driveToWall(1.0, 0.2, 32.0, 15000, false);
        if(isStopRequested())
        {
            return;
        }
        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobotToAngle(0.7, ROBOT_ANGLE + 95.0, 7000);
        driveAtHeadingForTime(-1.0, 0.4, DRIVE_ANGLE - 90.0, ROBOT_ANGLE + 85.0, 150);
        shoot(2000);
        robot.setShooterSpeed(0.0);
        // Check to see if the program should exit
        if(isStopRequested())
        {
            return;
        }

        driveAtHeadingForTime(1.0, 0.4, DRIVE_ANGLE, AUTO_ANGLE, 500);
        if(isStopRequested())
        {
            return;
        }

        // Rotate the robot to the far beacon drive angle.
        rotateRobotToAngle(0.7, ROBOT_ANGLE + AUTO_ANGLE, 7000);

        driveToWall(1.0, 0.2, 8.0, 15000, false);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        // Rotate the robot to the far beacon drive angle.
        rotateRobotToAngle(0.7, ROBOT_ANGLE, 7000);
        // Check to see if the program should exit
        if(isStopRequested()) {
            return;
        }

        driveDistanceSidewaysOnHeading(1.0, 21.0, 7000, false);
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

        // Move towards the wall a distance we should pick up the line and beacon colors
        driveToWall(1.0, 0.2, 24.0, 15000, false);
        if(isStopRequested())
        {
            return;
        }

        driveAtHeadingForTime(1.0, 0.2, ROBOT_ANGLE, ROBOT_ANGLE, 500);
        if(isStopRequested())
        {
            return;
        }
        spinRobot(-0.5, 90.0, 5000);
        if(isStopRequested())
        {
            return;
        }
        driveAtHeadingForTime(1.0, 0.2, ROBOT_ANGLE, ROBOT_ANGLE - 90.0, 750);

        endAuto();
    }
}
