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
        final double WALL_DISTANCE = 44.0;
        setupRobotParameters(4, 40);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        waitForStart();

        sleep(10000);

        robot.resetGyro();

        driveToWall(1.0, 0.2, WALL_DISTANCE, 10000, true);
        if(isStopRequested())
        {
            return;
        }

        rotateRobotToAngle(1.0, 0.0, 1000);
        if(isStopRequested())
        {
            return;
        }

        // This is probably the wrong direction
        driveDistanceSidewaysOnHeading(-1.0, 51.0, 10000, false);
        if(isStopRequested())
        {
            return;
        }
        driveToWall(1.0, 0.2, WALL_DISTANCE, 10000, true);
        if(isStopRequested())
        {
            return;
        }
        rotateRobotToAngle(1.0, 0.0, 1000);
        if(isStopRequested())
        {
            return;
        }

        // Fire up the shooters, and rotate the robot to shooting angle
        robot.setShooterSpeed(HardwareOmnibot.LOW_SHOOT_SPEED);
        rotateRobotToAngle(0.6, 10.0, 7000);
        shoot(2000);
        if(isStopRequested())
        {
            return;
        }
        robot.setShooterSpeed(0.0);

        rotateRobotToAngle(0.6, 110.0, 7000);
        if(isStopRequested())
        {
            return;
        }
        endAuto();
    }
}
