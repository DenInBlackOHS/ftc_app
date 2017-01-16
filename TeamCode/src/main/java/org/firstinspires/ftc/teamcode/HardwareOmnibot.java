package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Thread;

import static java.lang.Math.*;

/**
 *Created by Ethan
 */
public class HardwareOmnibot
{
    /* Public OpMode members. */
    // Shooter speeds
    public final static double HIGH_SHOOT_SPEED = 0.95;
    public final static double MID_HIGH_SHOOT_SPEED = 0.90;
    public final static double MID_LOW_SHOOT_SPEED = 0.85;
    public final static double LOW_SHOOT_SPEED = 0.80;
    public final static double MAX_SPIN_RATE = 0.6;
    public final static double MIN_SPIN_RATE = 0.1;
    public final static double MIN_DRIVE_RATE = 0.1;
    public final static double LIFT_SPEED = 0.5;
    public final static double SWEEP_SPEED = 0.75;

    // Color sensor threshold value
    public final static int MIN_BLUE_COLOR_VALUE = 2;
    public final static int MIN_RED_COLOR_VALUE = 2;
    public final static double MIN_ODS_VALUE = 0.01;

    // Robot Controller Config Strings
    public final static String FRONT_COLOR_SENSOR = "front_color";
    public final static String BACK_COLOR_SENSOR = "back_color";
    public final static String BACK_RANGE_SENSOR = "back_range";
    public final static String FRONT_RANGE_SENSOR = "front_range";
    public final static String GYRO = "gyro";
    public final static String FRONT_LEFT_MOTOR = "MotorLF";
    public final static String FRONT_RIGHT_MOTOR = "MotorRF";
    public final static String BACK_LEFT_MOTOR = "MotorLR";
    public final static String BACK_RIGHT_MOTOR = "MotorRR";
    public final static String SWEEPER_MOTOR = "SweeperMotor";
    public final static String LIFT_MOTOR = "LiftMotor";
    public final static String SHOOTER_1_MOTOR = "Shoot1";
    public final static String SHOOTER_2_MOTOR = "Shoot2";
    public final static String BUTTON_SERVO = "ServoButton";

    protected DcMotor leftMotorFore = null;
    protected DcMotor rightMotorFore = null;
    protected DcMotor leftMotorRear = null;
    protected DcMotor rightMotorRear = null;
    protected DcMotor sweeperMotor = null;
    protected DcMotor liftMotor = null;
    protected DcMotor shootMotor1 = null;
    protected DcMotor shootMotor2 = null;
    protected Servo buttonPush = null;
    protected ModernRoboticsI2cGyro gyro = null;
    protected ModernRoboticsAnalogOpticalDistanceSensor ods1 = null;
    public double ambientLight = 0;

    // Code to allow disabling the color sensors for teleop
    protected ModernRoboticsI2cColorSensor colorSensorFront = null;
    protected ModernRoboticsI2cColorSensor colorSensorBack = null;
    private I2cAddr frontColorAddress  = ModernRoboticsI2cColorSensor.DEFAULT_I2C_ADDRESS;
    private I2cAddr backColorAddress = I2cAddr.create8bit(ModernRoboticsI2cColorSensor.DEFAULT_I2C_ADDRESS.get8Bit() + 0x10);
    private I2cController frontColorController;
    private I2cController backColorController;
    private I2cController.I2cPortReadyCallback frontColorCallback;
    private I2cController.I2cPortReadyCallback backColorCallback;
    private boolean colorSensorsDisabled = false;

    // Code to allow disabling the color sensors for teleop
    protected ModernRoboticsI2cRangeSensor rangeSensorBack = null;
    private I2cAddr backRangeAddress = I2cAddr.create8bit(ModernRoboticsI2cRangeSensor.ADDRESS_I2C_DEFAULT.get8Bit() + 0x10);
    private I2cDeviceSynch rangeDeviceBack = null;
    // Seems like 2 range sensors just tend to interfere with each other.
//    protected ModernRoboticsI2cRangeSensor rangeSensorFront = null;
//    private I2cAddr frontRangeAddress = ModernRoboticsI2cRangeSensor.ADDRESS_I2C_DEFAULT;
//    private I2cDeviceSynch rangeDeviceFront = null;
    private boolean rangeSensorsDisabled = false;

    private static final int encoderClicksPerSecond = 2800;
    private double leftForeMotorPower = 0.0;
    private double leftRearMotorPower = 0.0;
    private double rightForeMotorPower = 0.0;
    private double rightRearMotorPower = 0.0;
    private double shooterSpeed = 0.0;
    private double liftMotorPower = 0.0;
    private double sweeperMotorPower = 0.0;

    /* local OpMode members. */
    private HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibot(){

    }

    public void initColorSensors()
    {
        colorSensorFront   = hwMap.get(ModernRoboticsI2cColorSensor.class, FRONT_COLOR_SENSOR);
        colorSensorFront.setI2cAddress(frontColorAddress);
        frontColorController = colorSensorFront.getI2cController();
        frontColorCallback = frontColorController.getI2cPortReadyCallback(colorSensorFront.getPort());

        colorSensorBack  = hwMap.get(ModernRoboticsI2cColorSensor.class, BACK_COLOR_SENSOR);
        colorSensorBack.setI2cAddress(backColorAddress);
        backColorController = colorSensorBack.getI2cController();
        backColorCallback =  backColorController.getI2cPortReadyCallback(colorSensorBack.getPort());

        colorSensorsDisabled = false;
    }

    public void enableColorSensors()
    {
        if(colorSensorsDisabled) {
            if (frontColorCallback != null)
                frontColorController.registerForI2cPortReadyCallback(frontColorCallback, colorSensorFront.getPort());
            if (backColorCallback != null)
                backColorController.registerForI2cPortReadyCallback(backColorCallback, colorSensorBack.getPort());
        }
        colorSensorsDisabled = false;
    }

    public void disableColorSensors()
    {
        if (!colorSensorsDisabled) {
            frontColorController.deregisterForPortReadyCallback(colorSensorFront.getPort());
            backColorController.deregisterForPortReadyCallback(colorSensorBack.getPort());
        }
        colorSensorsDisabled = true;
    }

    public double readRedFrontColorSensor() {
        int blueValue = 0;
        int redValue = 0;
        int result = 0;
        if(!colorSensorsDisabled) {
            redValue = colorSensorFront.red();
            blueValue = colorSensorFront.blue();
        }

        if(redValue >= blueValue) {
            result = redValue;
        }
        return result;
    }

    public double readBlueFrontColorSensor() {
        int blueValue = 0;
        int redValue = 0;
        int result = 0;
        if(!colorSensorsDisabled) {
            redValue = colorSensorFront.red();
            blueValue = colorSensorFront.blue();
        }

        if(blueValue >= redValue) {
            result = blueValue;
        }
        return result;
    }

    public double readRedBackColorSensor() {
        int blueValue = 0;
        int redValue = 0;
        int result = 0;
        if(!colorSensorsDisabled) {
            redValue = colorSensorBack.red();
            blueValue = colorSensorBack.blue();
        }

        if(redValue >= blueValue) {
            result = redValue;
        }
        return result;
    }

    public double readBlueBackColorSensor() {
        int blueValue = 0;
        int redValue = 0;
        int result = 0;
        if(!colorSensorsDisabled) {
            redValue = colorSensorBack.red();
            blueValue = colorSensorBack.blue();
        }

        if(blueValue >= redValue) {
            result = blueValue;
        }
        return result;
    }

    public void initRangeSensors()
    {
        rangeDeviceBack = (I2cDeviceSynch)hwMap.get(BACK_RANGE_SENSOR);
        rangeDeviceBack.setI2cAddress(backRangeAddress);
        rangeSensorBack = new ModernRoboticsI2cRangeSensor(rangeDeviceBack);
        rangeSensorBack.setI2cAddress(backRangeAddress);

//        rangeDeviceFront = (I2cDeviceSynch)hwMap.get(FRONT_RANGE_SENSOR);
//        rangeDeviceFront.setI2cAddress(frontRangeAddress);
//        rangeSensorFront = new ModernRoboticsI2cRangeSensor(rangeDeviceFront);
//        rangeSensorFront.setI2cAddress(frontRangeAddress);
        rangeSensorsDisabled = false;
    }

    public void enableRangeSensors()
    {
        if(rangeSensorsDisabled) {
            rangeDeviceBack.engage();
//            rangeDeviceFront.engage();
        }
        rangeSensorsDisabled = false;
    }

    public void disableRangeSensors()
    {
        if (!rangeSensorsDisabled) {
            rangeDeviceBack.disengage();
//            rangeDeviceFront.disengage();
        }
        rangeSensorsDisabled = true;
    }

    public double readBackRangeSensor() {
        double result = 0.0;
        if(!rangeSensorsDisabled) {
            result = rangeSensorBack.getDistance(DistanceUnit.MM);
        }
        return result;
    }

//    public double readFrontRangeSensor() {
//        double result = 0.0;
//        if(!rangeSensorsDisabled) {
//            result = rangeSensorFront.getDistance(DistanceUnit.MM);
//        }
//        return result;
//    }

    public void initOds()
    {
        ods1 = hwMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "ods1");
        ambientLight = ods1.getLightDetected();
    }

    public double readOds()
    {
        return (ods1.getLightDetected() - ambientLight);
    }

    public void initGyro()
    {
        // Init Gyro Code
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, GYRO);
        gyro.setI2cAddress(I2cAddr.create8bit(0x20));
    }

    public void calibrateGyro()
    {
        // Calibrate Gyro Code
        gyro.calibrate();
        while(gyro.isCalibrating()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
        }
    }

    public void resetGyro()
    {
        // Reset Gyro Code
        gyro.resetZAxisIntegrator();
    }

    public double readGyro()
    {
        // Read Gyro Code
        double heading = (double)gyro.getHeading();
        heading = abs(heading - 360.0);
        return heading;
    }

    public void setShooterSpeed(double speed)
    {
        if(speed != shooterSpeed)
        {
            shooterSpeed = speed;
            shootMotor1.setPower(speed);
            shootMotor2.setPower(speed);
        }
    }

    public void setLeftForeMotorPower(double power)
    {
        if(power != leftForeMotorPower)
        {
            leftForeMotorPower = power;
            leftMotorFore.setPower(power);
        }
    }

    public void setLeftRearMotorPower(double power)
    {
        if(power != leftRearMotorPower)
        {
            leftRearMotorPower = power;
            leftMotorRear.setPower(power);
        }
    }

    public void setRightForeMotorPower(double power)
    {
        if(power != rightForeMotorPower)
        {
            rightForeMotorPower = power;
            rightMotorFore.setPower(power);
        }
    }

    public void setRightRearMotorPower(double power)
    {
        if(power != rightRearMotorPower)
        {
            rightRearMotorPower = power;
            rightMotorRear.setPower(power);
        }
    }

    public void setAllDriveZero()
    {
        setLeftForeMotorPower(0.0);
        setLeftRearMotorPower(0.0);
        setRightForeMotorPower(0.0);
        setRightRearMotorPower(0.0);
    }

    public void setLiftMotorPower(double speed)
    {
        if(speed != liftMotorPower)
        {
            liftMotorPower = speed;
            liftMotor.setPower(speed);
        }
    }

    public void setSweeperMotorPower(double speed)
    {
        if(speed != sweeperMotorPower)
        {
            sweeperMotorPower = speed;
            sweeperMotor.setPower(speed);
        }
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset)
    {
        // Read Gyro Angle Here
        double reducedSpin = spin * MAX_SPIN_RATE;
        double gyroAngle = readGyro() + angleOffset;
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(315.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(225.0 + gyroAngle);

        if(abs(yPower) < MIN_DRIVE_RATE)
        {
            yPower = 0.0;
        }
        if(abs(xPower) < MIN_DRIVE_RATE)
        {
            xPower = 0.0;
        }
        if(abs(spin) < MIN_SPIN_RATE)
        {
            reducedSpin = 0.0;
        }

        double LFpower = (xPower * cos(leftFrontAngle) + yPower * sin(leftFrontAngle))/sqrt(2) + reducedSpin;
        double LRpower = (xPower * cos(leftRearAngle) + yPower * sin(leftRearAngle))/sqrt(2) + reducedSpin;
        double RFpower = (xPower * cos(rightFrontAngle) + yPower * sin(rightFrontAngle))/sqrt(2) + reducedSpin;
        double RRpower = (xPower * cos(rightRearAngle) + yPower * sin(rightRearAngle))/sqrt(2) + reducedSpin;

        double maxPower = max(1.0, max(max(LFpower, LRpower),
                max(RFpower, RRpower)));

        LFpower /= maxPower;
        RFpower /= maxPower;
        RFpower /= maxPower;
        RRpower /= maxPower;

        setLeftForeMotorPower(LFpower);
        setLeftRearMotorPower(LRpower);
        setRightForeMotorPower(RFpower);
        setRightRearMotorPower(RRpower);
    }

    public void resetDriveEncoders()
    {
        int sleepTime = 0;
        int encoderCount = leftMotorFore.getCurrentPosition();

        rightMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFore.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
            }
            sleepTime += 10;
            encoderCount = leftMotorFore.getCurrentPosition();
        }

        leftMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFore.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotorFore = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        rightMotorFore  = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        leftMotorRear = hwMap.dcMotor.get(BACK_LEFT_MOTOR);
        rightMotorRear = hwMap.dcMotor.get(BACK_RIGHT_MOTOR);
        sweeperMotor = hwMap.dcMotor.get(SWEEPER_MOTOR);
        liftMotor = hwMap.dcMotor.get(LIFT_MOTOR);
        shootMotor1 = hwMap.dcMotor.get(SHOOTER_1_MOTOR);
        shootMotor2 = hwMap.dcMotor.get(SHOOTER_2_MOTOR);
        buttonPush = hwMap.servo.get(BUTTON_SERVO);

        leftMotorFore.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFore.setDirection(DcMotor.Direction.FORWARD);
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD);
        rightMotorRear.setDirection(DcMotor.Direction.FORWARD);

        sweeperMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor1.setDirection(DcMotor.Direction.FORWARD);
        shootMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        setAllDriveZero();
        setSweeperMotorPower(0.0);
        setLiftMotorPower(0.0);
        setShooterSpeed(0.0);

        resetDriveEncoders();
        shootMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
// Until we can set the motor parameters in the motor controller, this slows the robot down.
// Need to update the motor controller firmware to do this.
//        leftMotorFore.setMaxSpeed(encoderClicksPerSecond);
//        rightMotorFore.setMaxSpeed(encoderClicksPerSecond);
//        leftMotorRear.setMaxSpeed(encoderClicksPerSecond);
//        rightMotorRear.setMaxSpeed(encoderClicksPerSecond);
        shootMotor1.setMaxSpeed(encoderClicksPerSecond);
        shootMotor2.setMaxSpeed(encoderClicksPerSecond);

        initGyro();
        initOds();
        initColorSensors();
        initRangeSensors();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    /*
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    */
}

