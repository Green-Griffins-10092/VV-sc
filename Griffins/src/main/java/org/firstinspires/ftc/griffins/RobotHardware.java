package org.firstinspires.ftc.griffins;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.griffins.Navigation.PIDController;
import org.firstinspires.ftc.robotcore.external.Func;

import java.util.concurrent.locks.Lock;

import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE_BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.BLUE_RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED_BLUE;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.RED_RED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.UNDEFINED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.UNDEFINED_UNDEFINED;
import static org.firstinspires.ftc.griffins.RobotHardware.BeaconState.guessBeaconState;

/**
 * Created by David on 10/29/2016.
 * Stores the hardware for the robot.
 */
public class RobotHardware {

    // Hardware map name constants
    public static final String LEFT_DRIVE_ONE = "left1";
    public static final String LEFT_DRIVE_TWO = "left2";
    public static final String RIGHT_DRIVE_ONE = "right1";
    public static final String RIGHT_DRIVE_TWO = "right2";
    public static final String SHOOTER_MOTOR_LEFT = "shooterL";
    public static final String SHOOTER_MOTOR_RIGHT = "shooterR";
    public static final String INTAKE_MOTOR = "intake";
    public static final String TURRET_ROTATION_MOTOR = "turret";
    public static final String LEFT_TURRET_GUIDE_SERVO = "turretL";
    public static final String RIGHT_TURRET_GUIDE_SERVO = "turretR";
    public static final String BUTTON_PUSHER_SERVO = "button";
    public static final String LOADER_SERVO_ONE = "loader";
    public static final String LOADER_SERVO_TWO = "loader2";
    public static final String TURRET_GYRO = "gyro";
    public static final String LEFT_BUTTON_PUSHER_SENSOR = "colorL";
    public static final String RIGHT_BUTTON_PUSHER_SENSOR = "colorR";
    public static final String LOADER_COLOR_SENSOR = "intake color";
    public static final String BEACON_DISTANCE_SENSOR = "distance";
    public static final String LOADER_SWITCH = "loader switch";
    public static final String BNO055_SENSOR = "bno055";

    // The constants for motor encoders
    public static final int NEVEREST_ENCODER_COUNT_PER_ROTATION = 28;
    public static final int NEVEREST_40_ENCODER_COUNTS_PER_ROTATION = NEVEREST_ENCODER_COUNT_PER_ROTATION * 40;
    // The addresses for the color sensors, since we are using three, the default will be changed
    public static final I2cAddr LEFT_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x38);
    public static final I2cAddr RIGHT_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x3C);
    public static final I2cAddr LOADER_COLOR_SENSOR_ADDRESS = I2cAddr.create8bit(0x32);
    // The constants for the button pusher positions
    public static final double BUTTON_PUSHER_CENTER_POSITION = 97 / 255.0;
    public static final double BUTTON_PUSHER_RATIO = 5 / 5.0;
    public static final double BUTTON_PUSHER_LEFT_FULL_EXTENSION = 67 / 255.0;
    public static final double BUTTON_PUSHER_RIGHT_FULL_EXTENSION = 127 / 255.0;
    public static final double BUTTON_PUSHER_RETRACTED = 20 / 255.0;
    public static final double BUTTON_PUSHER_EXTENDED = 80 / 255.0;
    // The constants for the loader speeds
    public static final double LOADER_ZERO_POWER = 0;
    public static final double LOADER_FULL_REVERSE_POWER = -2 / 3.0;
    public static final double LOADER_FULL_FORWARD_POWER = 2 / 3.0;
    // The constants for shooting speeds
    public static final double SHOOTER_SPEED = 0.9;
    // The constants for driving
    public static final double INCHES_PER_ENCODER_COUNT = (2 * Math.PI) / (NEVEREST_ENCODER_COUNT_PER_ROTATION * 10);  // (wheel diameter * pi) / (encoder counts per motor rotation * gear ratio)
    public static final double ENCODER_COUNTS_PER_INCH = 1 / INCHES_PER_ENCODER_COUNT; // inverse of above INCHES_PER_ENCODER_COUNT
    public static final double ENCODER_COUNTS_PER_ROBOT_REVOLUTION = RobotHardware.ENCODER_COUNTS_PER_INCH * Math.PI * 14.5625; // (pi * diameter) * encoder counts per inch, if the turn is on point
    public static final double ENCODER_COUNTS_PER_ROBOT_DEGREE = ENCODER_COUNTS_PER_ROBOT_REVOLUTION / 360; // multiply ENCODER_COUNTS_PER_ROBOT_ROTATION by 1rot/360deg
    //turret encoder limits
    public static final double ENCODER_COUNTS_PER_TURRET_REVOLUTION = NEVEREST_40_ENCODER_COUNTS_PER_ROTATION * 3;
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREE = ENCODER_COUNTS_PER_TURRET_REVOLUTION / 360;
    public static final int TURRET_ENCODER_COUNT_REVOLUTION_LIMIT = (int) (ENCODER_COUNTS_PER_TURRET_DEGREE * 200);
    // The Vuforia License Key
    public static final String VUFORIA_LICENSE_KEY = "AT3JPAj/////AAAAGdMIFxYU7UYXs7ZUAq3xlpUJDbeYIAIe69usc" +
            "Pw2c6g2kjczfs8x9A1YX2Mi3SLEFsx0JM1x9Lm733yP8I8HxGqUGJ" +
            "r2HDUcUheHtJZ3itjlnCaAZWYqeV+RB4bU8t3BW3pnecmQ9BmjcUO" +
            "aO32ENJIc0PRvpzPFNTtKB6HHJwNhPoMZYonmVEVeCwsuSfIhyzl1" +
            "KWHU8GVzgdz3NRBs0O7Dedd+cECw9dmXX0TutXkuMr9ykOstrDXM6" +
            "1D1Hb2DuY+4LKERkLFwUm/TDv5+zR7A4eDoE92nmEIpVdSfR7kNYG" +
            "QGeDbWK7/oHGjwVYOZvEvmTW9dMBDQNiCCeWCag6o4odFTMo5Tc8U6+grD2qVR";
    private PIDController turretController;
    private double BUTTON_PUSHER_LEFT_POSITION = (BUTTON_PUSHER_LEFT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) * BUTTON_PUSHER_RATIO + BUTTON_PUSHER_CENTER_POSITION;
    private double BUTTON_PUSHER_RIGHT_POSITION = (BUTTON_PUSHER_RIGHT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) * BUTTON_PUSHER_RATIO + BUTTON_PUSHER_CENTER_POSITION;
    private double BUTTON_PUSHER_EXTENDED_POSITION = (BUTTON_PUSHER_EXTENDED - BUTTON_PUSHER_RETRACTED) * BUTTON_PUSHER_RATIO + BUTTON_PUSHER_RETRACTED;
    private BeaconState alliance;
    //motor variables
    private SyncedDcMotors leftDrive;
    private SyncedDcMotors rightDrive;
    private SyncedDcMotors shooter;
    private DcMotor intake;
    private DcMotor turretRotation;
    //servo variables
    private Servo leftTurretGuide;
    private Servo rightTurretGuide;
    private Servo buttonPusherServo;
    //private CRServo loaderServoTwo;
    private Servo loaderServoOne;
    //sensor variables
    private ModernRoboticsI2cGyro turretGyro;
    private ModernRoboticsI2cColorSensor leftButtonPusherColorSensor;
    private ModernRoboticsI2cColorSensor rightButtonPusherColorSensor;
    private ModernRoboticsI2cColorSensor loaderColorSensor;
    private ModernRoboticsAnalogOpticalDistanceSensor beaconDistanceSensor;
    private DigitalChannel loaderParticleLimitSwitch;
    private BNO055IMU robotTracker;
    private double turretHeadingTarget;

    public RobotHardware() {
    }

    public void initialize(HardwareMap hardwareMap) {
        leftDrive = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.REVERSE, SyncedDcMotors.ALL_SAME, LEFT_DRIVE_ONE, LEFT_DRIVE_TWO);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.FORWARD, SyncedDcMotors.ALL_SAME, RIGHT_DRIVE_ONE, RIGHT_DRIVE_TWO);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new SyncedDcMotors(hardwareMap, DcMotorSimple.Direction.REVERSE, SyncedDcMotors.ALTERNATING, SHOOTER_MOTOR_LEFT, SHOOTER_MOTOR_RIGHT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretRotation = hardwareMap.get(DcMotor.class, TURRET_ROTATION_MOTOR);
        turretRotation.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        buttonPusherServo = hardwareMap.get(Servo.class, BUTTON_PUSHER_SERVO);
        buttonPusherServo.setDirection(Servo.Direction.FORWARD);
        this.pushButton(UNDEFINED);

        leftTurretGuide = hardwareMap.get(Servo.class, LEFT_TURRET_GUIDE_SERVO);
        leftTurretGuide.setDirection(Servo.Direction.FORWARD);

        rightTurretGuide = hardwareMap.get(Servo.class, RIGHT_TURRET_GUIDE_SERVO);
        rightTurretGuide.setDirection(Servo.Direction.REVERSE);

        setTurretGuidePosition(0);

        loaderServoOne = hardwareMap.get(Servo.class, LOADER_SERVO_ONE);
        loaderServoOne.setDirection(Servo.Direction.REVERSE);

        /*loaderServoTwo = hardwareMap.get(CRServo.class, LOADER_SERVO_TWO);
        loaderServoTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        loaderServoTwo.setPower(LOADER_ZERO_POWER);*/

        this.setLoaderPower(0);

        turretGyro = (ModernRoboticsI2cGyro) hardwareMap.get(GyroSensor.class, TURRET_GYRO);
        turretGyro.calibrate();  //look at z axis scaling coefficient when available
        turretGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN); // verify that the angles have the correct sign

        leftButtonPusherColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, LEFT_BUTTON_PUSHER_SENSOR);
        leftButtonPusherColorSensor.setI2cAddress(LEFT_COLOR_SENSOR_ADDRESS);
        leftButtonPusherColorSensor.enableLed(true);
        leftButtonPusherColorSensor.enableLed(false);

        rightButtonPusherColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, RIGHT_BUTTON_PUSHER_SENSOR);
        rightButtonPusherColorSensor.setI2cAddress(RIGHT_COLOR_SENSOR_ADDRESS);
        rightButtonPusherColorSensor.enableLed(true);
        rightButtonPusherColorSensor.enableLed(false);

        deregisterBeaconColorSensors(); // TODO: 2/12/2017 need to register the color sensors in relevant files!

        loaderColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, LOADER_COLOR_SENSOR);
        loaderColorSensor.setI2cAddress(LOADER_COLOR_SENSOR_ADDRESS);
        loaderColorSensor.enableLed(false);
        loaderColorSensor.enableLed(true);

        deregisterLoaderColorSensor();

        /*beaconDistanceSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, BEACON_DISTANCE_SENSOR);
        beaconDistanceSensor.enableLed(true);

        loaderParticleLimitSwitch = hardwareMap.get(DigitalChannel.class, LOADER_SWITCH);
        loaderParticleLimitSwitch.setMode(DigitalChannelController.Mode.INPUT);

        robotTracker = hardwareMap.get(BNO055IMU.class, BNO055_SENSOR);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelerationIntegrationAlgorithm = new GriffinAccelerationIntegratorLowPass();
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = false;
        robotTracker.initialize(parameters);
        robotTracker.startAccelerationIntegration(new Position(), new Velocity(), 10);*/
        turretController = new PIDController(0.07, 0, 0, 0.05, new Func<Double>() {
            @Override
            public Double value() {
                return (turretRotation.getCurrentPosition() / ENCODER_COUNTS_PER_TURRET_DEGREE - turretGyro.getIntegratedZValue());
            }
        }, null);

        turretController.setSetPoint(turretController.getSourceVal());
    }

    public SyncedDcMotors getLeftDrive() {
        return leftDrive;
    }

    public SyncedDcMotors getRightDrive() {
        return rightDrive;
    }

    public SyncedDcMotors getShooter() {
        return shooter;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public DcMotor getTurretRotation() {
        return turretRotation;
    }

    public void setTurretRotation(double joystickInput, boolean trackingOn) {
        if (this.getTurretGyro().isCalibrating()) {
            trackingOn = false;
        }
        double turretSpeed;

        if (trackingOn) {
            turretController.setSetPoint(turretController.getSetPoint() + joystickInput * 2);
            turretSpeed = turretController.sendPIDOutput();

        } else {
            turretSpeed = joystickInput;
            turretController.setSetPoint(turretController.getSourceVal());
        }

        turretRotation.setPower(turretSpeed);
    }

    @Deprecated
    public BNO055IMU getRobotTracker() {
        return robotTracker;
    }

    public ModernRoboticsI2cGyro getTurretGyro() {
        return turretGyro;
    }

    public ColorSensor getLeftButtonPusherColorSensor() {
        return leftButtonPusherColorSensor;
    }

    public ColorSensor getRightButtonPusherColorSensor() {
        return rightButtonPusherColorSensor;
    }

    public ModernRoboticsI2cColorSensor getLoaderColorSensor() {
        return loaderColorSensor;
    }

    @Deprecated
    public DigitalChannel getLoaderParticleLimitSwitch() {
        return loaderParticleLimitSwitch;
    }

    @Deprecated
    public void setTurretGuidePosition(double position) {
        // TODO: 10/29/2016 create code for turret position
    }

    public void setDrivePower(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void stopDrive() {
        setDrivePower(0, 0);
    }

    public void extendButtonPusher(double percentExtension) {
        changeButtonPusherExtension(percentExtension);
        buttonPusherServo.setPosition(BUTTON_PUSHER_EXTENDED_POSITION);
    }

    public void retractButtonPusher() {
        buttonPusherServo.setPosition(BUTTON_PUSHER_RETRACTED);
    }

    /**
     * This method will operate the button pusher servo to push the beacon.
     *
     * @param beaconState      encodes the state of the beacon.  If one side is undefined,
     *                         it will assume that side is the opposite of the defined side.
     *                         If UNDEFINED is passed, it will return to the center position.
     * @param alliance         stores what alliance we are, valid parameters are BLUE and RED
     * @param percentExtension is what percent of the button pusher's range will be used.
     */
    @Deprecated
    public void pushButton(BeaconState beaconState, BeaconState alliance, double percentExtension) {
        changeButtonPusherExtension(percentExtension);
        double BUTTON_PUSHER_LEFT_POSITION = this.BUTTON_PUSHER_LEFT_POSITION;
        double BUTTON_PUSHER_CENTER_POSITION = RobotHardware.BUTTON_PUSHER_CENTER_POSITION;
        double BUTTON_PUSHER_RIGHT_POSITION = this.BUTTON_PUSHER_RIGHT_POSITION;


        if (beaconState == UNDEFINED || beaconState == UNDEFINED_UNDEFINED) {
            buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
        } else {
            beaconState = guessBeaconState(beaconState);
            if (alliance == BLUE) {
                if (beaconState == BLUE_RED) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
                } else if (beaconState == RED_BLUE) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
                } else if (beaconState == BLUE_BLUE) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
                } else if (beaconState == RED_RED) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
                }
            } else if (alliance == RED) {
                if (beaconState == BLUE_RED) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_RIGHT_POSITION);
                } else if (beaconState == RED_BLUE) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
                } else if (beaconState == BLUE_BLUE) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_LEFT_POSITION);
                } else if (beaconState == RED_RED) {
                    buttonPusherServo.setPosition(BUTTON_PUSHER_CENTER_POSITION);
                }
            }
        }
    }

    /**
     * This method will operate the button pusher servo to push the beacon.
     *
     * @param beaconState encodes the state of the beacon.  If one side is undefined,
     *                    it will assume that side is the opposite of the defined side.
     *                    If UNDEFINED is passed, it will return to the center position
     * @param alliance    stores what alliance we are, valid parameters are BLUE and RED
     */
    @Deprecated
    public void pushButtonFullExtension(BeaconState beaconState, BeaconState alliance) {
        pushButton(beaconState, alliance, 1);
    }

    @Deprecated
    public void pushButton(BeaconState beaconState, BeaconState alliance) {
        pushButton(beaconState, alliance, BUTTON_PUSHER_RATIO);
    }

    @Deprecated
    public void pushButton(BeaconState beaconState) {
        pushButton(beaconState, BLUE);
    }

    public void deregisterBeaconColorSensors() {
        leftButtonPusherColorSensor.getI2cController().deregisterForPortReadyCallback(leftButtonPusherColorSensor.getPort());
        rightButtonPusherColorSensor.getI2cController().deregisterForPortReadyCallback(rightButtonPusherColorSensor.getPort());
    }

    public void registerBeaconColorSensors() {
        leftButtonPusherColorSensor.getI2cController().registerForI2cPortReadyCallback(leftButtonPusherColorSensor, leftButtonPusherColorSensor.getPort());
        rightButtonPusherColorSensor.getI2cController().registerForI2cPortReadyCallback(rightButtonPusherColorSensor, rightButtonPusherColorSensor.getPort());
    }

    public void deregisterLoaderColorSensor() {
        loaderColorSensor.getI2cController().deregisterForPortReadyCallback(loaderColorSensor.getPort());
    }

    public void registerLoaderColorSensor() {
        loaderColorSensor.getI2cController().registerForI2cPortReadyCallback(loaderColorSensor, loaderColorSensor.getPort());
    }

    /**
     * Will check the color sensors to determine the beacon state
     * If either color sensor's state is UNDEFINED, then this method will return UNDEFINED.
     *
     * @return the state of the beacon, as a variable of BeaconState,
     */
    @Deprecated
    public BeaconState findBeaconState() {
        BeaconState leftSide = findColorSensorState(leftButtonPusherColorSensor);
        BeaconState rightSide = findColorSensorState(rightButtonPusherColorSensor);

        return BeaconState.mergeBeaconStates(leftSide, rightSide);
    }

    public BeaconState findLeftBeaconState() {
        return findColorSensorState(leftButtonPusherColorSensor);
    }

    public BeaconState findRightBeaconState() {
        return findColorSensorState(rightButtonPusherColorSensor);
    }

    public BeaconState findParticleColor() { //unfortunately, a blue ball returns equal values for blue and green
        RobotHardware.BeaconState colorState = UNDEFINED;

        /*if (loaderColorSensor.red() > loaderColorSensor.blue() + 5 && loaderColorSensor.alpha() > 10) {
            colorState = RED;
        } else if (loaderColorSensor.blue() > loaderColorSensor.red() + 5 && loaderColorSensor.alpha() > 10) {
            colorState = BLUE;
        }*/

        int colorNumber = getLoaderColorNumber();

        if (colorNumber > 0 && colorNumber < 16 && colorNumber != 14) { //reason tests
            if (colorNumber < 7 || colorNumber == 15) {
                colorState = BLUE;
            } else if (colorNumber > 8 || colorNumber == 13) {
                colorState = RED;
            }
        }

        return colorState;
    }

    public int getLoaderColorNumber() {
        I2cController controller = loaderColorSensor.getI2cController();

        byte color = -1;
        Lock readLock = controller.getI2cReadCacheLock(loaderColorSensor.getPort());
        byte[] readBuffer = controller.getI2cReadCache(loaderColorSensor.getPort());
        try {
            readLock.lock();
            color = readBuffer[ModernRoboticsI2cColorSensor.OFFSET_COLOR_NUMBER];
        } finally {
            readLock.unlock();
        }

        if (color == -1)
            return color;
        return TypeConversion.unsignedByteToInt(color);
    }

    /**
     * Checks the state of the color sensor to determine what color is being read
     *
     * @param colorSensor the color sensor that will be checked
     * @return A BeaconState, which will be either RED, BLUE, or UNDEFINED.
     */
    private BeaconState findColorSensorState(ColorSensor colorSensor) {
        BeaconState colorState = UNDEFINED;

        if (colorSensor.alpha() > 1) {
            if (colorSensor.red() > colorSensor.blue() + 1 && colorSensor.red() > colorSensor.green()) {
                colorState = RED;
            } else if (colorSensor.blue() > colorSensor.red() + 1 && colorSensor.blue() > colorSensor.green()) {
                colorState = BLUE;
            }
        }

        return colorState;
    }


    public void setLoaderPower(double power) {
        power = Range.clip(power, -1, 1);
        power = Range.scale(power, -1, 1, 0, 1);
        loaderServoOne.setPosition(power);
        //loaderServoTwo.setPower(power);
    }

    public void startTurretTracking() {
        turretRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretGyro.resetZAxisIntegrator();
        turretController.setSetPoint(0);
        turretHeadingTarget = 0;
        turretRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void changeButtonPusherExtension(double newRatio) {
        if (newRatio < 0 || newRatio > 1) {
            throw new IllegalArgumentException("The Ratio must be between 0 and 1");
        }

        BUTTON_PUSHER_LEFT_POSITION = (BUTTON_PUSHER_LEFT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) * newRatio + BUTTON_PUSHER_CENTER_POSITION;
        BUTTON_PUSHER_RIGHT_POSITION = (BUTTON_PUSHER_RIGHT_FULL_EXTENSION - BUTTON_PUSHER_CENTER_POSITION) * newRatio + BUTTON_PUSHER_CENTER_POSITION;


    }

    public enum BeaconState {
        BLUE(0b00_01), //0  0000
        RED(0b00_10), //1  0001
        UNDEFINED(0b00_11), //2  0010
        BLUE_BLUE(0b01_01), //3  0011
        BLUE_RED(0b01_10), //4  0100
        BLUE_UNDEFINED(0b01_11), //5  0101
        RED_BLUE(0b10_01), //6  0110
        RED_RED(0b10_10), //7  0111
        RED_UNDEFINED(0b10_11), //8  1000
        UNDEFINED_BLUE(0b11_01), //9  1001
        UNDEFINED_RED(0b11_10), //10 1010
        UNDEFINED_UNDEFINED(0b11_11); //11 1011

        private final int numberState;

        BeaconState(int numberState) {
            this.numberState = numberState;
        }

        public static BeaconState mergeBeaconStates(BeaconState left, BeaconState right) {
            if (left.numberState >> 2 != 0 || right.numberState >> 2 != 0) {
                throw new IllegalArgumentException("Valid arguments are BLUE, RED, UNDEFINED, all others are already merged");
            }

            return getFromNumberState((left.numberState << 2) + right.numberState);
        }

        public static boolean containsUndefined(BeaconState beaconState) {
            return ((beaconState.numberState >> 2) ^ 0b11) == 0 || ((beaconState.numberState & 0b11) ^ 0b11) == 0;
        }

        public static BeaconState removeUndefined(BeaconState containsUndefined) {
            if (containsUndefined.numberState >> 2 == UNDEFINED.numberState) {
                return getFromNumberState(containsUndefined.numberState & 0b00_11);
            } else {
                return getFromNumberState((containsUndefined.numberState & 0b11_00) >> 2);
            }
        }

        public static BeaconState guessBeaconState(BeaconState containsUndefined) {
            if (containsUndefined.numberState <= 0b00_11 || containsUndefined == UNDEFINED_UNDEFINED) {
                throw new IllegalArgumentException("Insufficient data");
            }

            if ((containsUndefined.numberState & 0b00_11) == UNDEFINED.numberState) {
                return getFromNumberState((removeUndefined(containsUndefined).numberState << 2) + (removeUndefined(containsUndefined).numberState ^ 0b11));
            } else if ((containsUndefined.numberState & 0b11_00) == (UNDEFINED.numberState << 2)) {
                return getFromNumberState(((removeUndefined(containsUndefined).numberState ^ 0b11) << 2) + removeUndefined(containsUndefined).numberState);
            } else {
                return containsUndefined;
            }
        }

        public static BeaconState getFromNumberState(int numberState) {
            switch (numberState) {
                case 0b00_01:
                    return BLUE;
                case 0b00_10:
                    return RED;
                case 0b00_11:
                    return UNDEFINED;
                case 0b01_01:
                    return BLUE_BLUE;
                case 0b01_10:
                    return BLUE_RED;
                case 0b01_11:
                    return BLUE_UNDEFINED;
                case 0b10_01:
                    return RED_BLUE;
                case 0b10_10:
                    return RED_RED;
                case 0b10_11:
                    return RED_UNDEFINED;
                case 0b11_01:
                    return UNDEFINED_BLUE;
                case 0b11_10:
                    return UNDEFINED_RED;
                case 0b11_11:
                    return UNDEFINED_UNDEFINED;
                default:
                    throw new IllegalArgumentException("Not a valid number state.");
            }
        }

        public boolean containsUndefined() {
            return containsUndefined(this);
        }

        public int numberState() {
            return numberState;
        }
    }
}
