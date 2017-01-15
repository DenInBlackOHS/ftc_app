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
        final double ROBOT_ANGLE = 0.0;
        final double DRIVE_ANGLE = -90.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        sleep(10000);

        robot.resetGyro();

        driveToWall(1.0, 0.2, 50.0, 5000, true);
        if(isStopRequested())
        {
            return;
        }

        rotateRobotToAngle(0.6, 270.0, 7000);
        if(isStopRequested())
        {
            return;
        }

        driveToWall(1.0, 0.2, 40.0, 5000, true);
        if(isStopRequested())
        {
            return;
        }
        // Fire up the shooters, and rotate the robot 90 degrees
        robot.setShooterSpeed(HardwareOmnibot.MID_LOW_SHOOT_SPEED);
        rotateRobotToAngle(0.6, 270.0, 7000);
        shoot(2000);
        if(isStopRequested())
        {
            return;
        }
        robot.setShooterSpeed(0.0);

        rotateRobotToAngle(0.6, 180.0, 7000);
        if(isStopRequested())
        {
            return;
        }
        // This should knock the cap ball off, and set the robot on the pedestal
        driveAtHeadingForTime(1.0, 0.2, 180.0, 250.0, 1000);
        if(isStopRequested())
        {
            return;
        }

        rotateRobotToAngle(0.6, 90.0, 7000);
        endAuto();
    }
}
