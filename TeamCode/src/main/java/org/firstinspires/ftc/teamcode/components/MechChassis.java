package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import static java.lang.Thread.sleep;

/**
 * Swerve chassis consists of 4 wheels with a Servo and DcMotor attached to each.
 * Track (distance between left and right wheels), wheelbase (distance between front and back wheels)
 * and wheel radius are adjustable.
 * Expected hardware configuration is:<br />
 * Servos: servoFrontLeft, servoFrontRight, servoBackLeft, servoBackRight.<br />
 * Motors: motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight.<br />
 * Orientation sensors (optional): imu, imu2
 */
public class MechChassis extends Logger<MechChassis> implements Configurable {

    final private CoreSystem core;

    public enum DriveMode {
        STOP,      // not moving
        STRAIGHT,  // driving in a straight line utilizing orientation sensor to correct itself,
        //  all servos are set to the same position
        ROTATE,    // rotating in place, all servos are set on tangent lines to a circle drawn
        //  through the centers of 4 wheels.
        STEER      // motor power and servos in the direction of movement are controlled by
        //  the driver; opposite servos are in central position
    }

    // distance between the centers of left and right wheels, inches
    private double track = 11.5;
    // distance between the centers of front and back wheels, inches
    private double wheelBase = 10.7;
    // wheel radius, inches
    private double wheelRadius = 2.0;
    // minimum power that should be applied to the wheel motors for robot to start moving
    private double minPower = 0.15;
    // maximum power that should be applied to the wheel motors
    private double maxPower = 0.99;
    private double maxRange = 127; // max range sensor detectable
    private double powerScale = 0.5;

    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;

    // array contains the same wheel assemblies as above variables
    //  and is convenient to use when actions have to be performed on all 4
    public CombinedOrientationSensor orientationSensor;

    private DriveMode driveMode = DriveMode.STOP;      // current drive mode
    private double targetHeading;     // intended heading for DriveMode.STRAIGHT as reported by orientation sensor
    private double headingDeviation;  // current heading deviation for DriveMode.STRAIGHT as reported by orientation sensor
    private double servoCorrection;   // latest correction applied to leading wheels' servos to correct heading deviation
    private double defaultScale = 0.8;
    private double curHeading = 0;
    private boolean useScalePower = true;//
    private boolean setImuTelemetry = false;//unless debugging, don't set telemetry for imu
    private boolean setRangeSensorTelemetry = false;//unless debugging, don't set telemetry for range sensor
    public double powerScale() { return powerScale; }


    //odometry motors
    DcMotorEx verticalLeftEncoder;
    DcMotorEx verticalRightEncoder;
    DcMotorEx horizontalEncoder;


    String rfName = "motorFR" , lfName = "motorFL";
    String rbName = "motorBR";
    String lbName = "motorBL";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;



    final double TICKS_PER_CM = 16.86;//number of encoder ticks per cm of driving

    public DcMotorEx verticalLeftEncoder(){ return verticalLeftEncoder; }
    public DcMotorEx verticalRightEncoder(){ return verticalRightEncoder; }
    public DcMotorEx horizontalEncoder(){ return horizontalEncoder; }

    public void enableRangeSensorTelemetry() { // must be call before reset() or setupTelemetry()
        setRangeSensorTelemetry = true;
    }

    public void enableImuTelemetry() {
        setImuTelemetry = true;
    }

    public double getDefaultScale() {
        return defaultScale;
    }

    public void setDefaultScale(double val) {
        defaultScale = val;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getTrack() {
        return track;
    }

    public void setTrack(double track) {
        this.track = track;
    }

    @Adjustable(min = 8.0, max = 18.0, step = 0.02)
    public double getWheelBase() {
        return wheelBase;
    }

    public void setWheelBase(double wheelBase) {
        this.wheelBase = wheelBase;
    }

    @Adjustable(min = 1.0, max = 5.0, step = 0.02)
    public double getWheelRadius() {
        return wheelRadius;
    }

    public void setWheelRadius(double wheelRadius) {
        this.wheelRadius = wheelRadius;
    }

    @Adjustable(min = 0.0, max = 0.4, step = 0.01)
    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    @Adjustable(min = 0.2, max = 1.0, step = 0.01)
    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    @Override
    public String getUniqueName() {
        return "chassis";
    }

    /**
     * SwerveChassis constructor
     */
    public MechChassis(CoreSystem core) {
        this.core = core;
    }

    /**
     * Used only for ToboRuckus, old code
     */
    @Deprecated
    public MechChassis() {
        this.core = new CoreSystem(); // Prevents null pointer exception
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    public void configure(Configuration configuration, boolean auto) {
        // set up motors / sensors as wheel assemblies

        motorFL = configuration.getHardwareMap().tryGet(DcMotorEx.class, lfName);
        motorFR = configuration.getHardwareMap().tryGet(DcMotorEx.class, rfName);
        motorBL = configuration.getHardwareMap().tryGet(DcMotorEx.class, lbName);
        motorBR = configuration.getHardwareMap().tryGet(DcMotorEx.class, rbName);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // map odometry encoders
        verticalLeftEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, verticalLeftEncoderName);
        verticalRightEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, verticalRightEncoderName);
        horizontalEncoder = configuration.getHardwareMap().tryGet(DcMotorEx.class, horizontalEncoderName);

        verticalLeftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //verticalLeftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        //verticalRightEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        if (auto || setImuTelemetry) {
            orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
            orientationSensor.configure(configuration.getHardwareMap(), "imu", "imu2");
        }

        // register chassis as configurable component
        configuration.register(this);
    }

    /**
     * move in the vertical direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void yMove(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(sgn * power);
    }

    /**
     * move in the horizontal direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void xMove(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(-sgn * power);
        motorBR.setPower(sgn * power);
    }

    public void freeStyle(double fl, double fr, double bl, double br) {
        motorFL.setPower(fl);
        motorFR.setPower(fr);
        motorBL.setPower(bl);
        motorBR.setPower(br);
    }

    /**
     * pivot and turn
     *
     * @param sgn   can be either +1 or -1(+1 for clockwise, -1 for counter clockwise)
     * @param power must be in range [0,1]
     */
    public void turn(int sgn, double power) {
        motorFL.setPower(sgn * power);
        motorFR.setPower(-sgn * power);
        motorBL.setPower(sgn * power);
        motorBR.setPower(-sgn * power);
    }

    /**
     * turning while driving
     *
     * @param power         must be in range [0,1]
     * @param turningFactor int range [-1,+1] (+1 for turning right, -1 for turning left)
     */
    public void carDrive(double power, double turningFactor) {

        if (turningFactor > 0) {
            turningFactor = 1 - turningFactor;
            if (power>0) { // drive forward
                motorFL.setPower(power);
                motorFR.setPower(turningFactor * power);
                motorBL.setPower(power);
                motorBR.setPower(turningFactor * power);
            } else {
                motorFL.setPower(turningFactor * power);
                motorFR.setPower(power);
                motorBL.setPower(turningFactor * power);
                motorBR.setPower(power);
            }
        } else {
            turningFactor = 1 + turningFactor;
            if (power>0) { // drive forward
                motorFL.setPower(turningFactor * power);
                motorFR.setPower(power);
                motorBL.setPower(turningFactor * power);
                motorBR.setPower(power);
            } else {
                motorFL.setPower(power);
                motorFR.setPower(turningFactor * power);
                motorBL.setPower(power);
                motorBR.setPower(turningFactor * power);
            }
        }

    }

    public void driveStraightAuto(double power, double cm, int timeout) throws InterruptedException {
        if (Thread.currentThread().isInterrupted()) return;

        if (power < 0 || power > 1) {
            throw new IllegalArgumentException("Power must be between 0 and 1");
        }

        double distance = TICKS_PER_CM * cm;

        if (power == 0) {
            //driveMode = SwerveChassis.DriveMode.STOP;
            targetHeading = 0;
            headingDeviation = 0;
            servoCorrection = 0;
            //for (SwerveChassis.WheelAssembly wheel : wheels) wheel.motor.setPower(0);
            orientationSensor.enableCorrections(false);
            return;
        }

        if (distance < 0) {
            power = -power;
            distance = -distance;
        }

        //motor settings
        //driveMode = SwerveChassis.DriveMode.STRAIGHT;
        int[] startingCount = new int[4];
        for (int i = 0; i < 4; i++) {
            //wheels[i].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //wheels[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //startingCount[i] = wheels[i].motor.getCurrentPosition();
        }

        //imu initialization
        orientationSensor.enableCorrections(true);
        targetHeading = orientationSensor.getHeading();

        //start powering wheels
        yMove(cm > 0 ? +1 : -1, power);
        //record time
        long iniTime = System.currentTimeMillis();

        //waiting loop
        while (true) {
            // check and correct heading as needed
            double sensorHeading = orientationSensor.getHeading();
            headingDeviation = targetHeading - sensorHeading;

            //determine if target distance is reached
            int maxTraveled = Integer.MIN_VALUE;
            for (int i = 0; i < 4; i++) {
                //maxTraveled = Math.abs(Math.max(maxTraveled, wheels[i].motor.getCurrentPosition() - startingCount[i]));
            }
            if (distance - maxTraveled < 10)
                break;
            //determine if time limit is reached
            if (System.currentTimeMillis() - iniTime > timeout)
                break;
            if (Thread.currentThread().isInterrupted())
                break;
            // yield handler
            this.core.yield();
        }
        stop();
        driveMode = DriveMode.STOP;
    }

    public void rotateTo(double power, double finalHeading) throws InterruptedException {
        rawRotateTo(power, finalHeading, true);//!!! A very bold move
        sleep(200);
        rawRotateTo(0.25, finalHeading, false);
    }

    private void rawRotateTo(double power, double finalHeading, boolean stopEarly) throws InterruptedException {
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        //do not turn if the heading is close enough the target
        if (Math.abs(deltaD) < 0.5)
            return;
        //resolve the issue with +-180 mark
        if (Math.abs(deltaD) > 180) {
            finalHeading = finalHeading + (deltaD > 0 ? -360 : +360);
            deltaD = 360 - Math.abs(deltaD);
            deltaD = -deltaD;
        }
        //break on reaching the target
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        turn((int) Math.signum(deltaD), power);
        //record heading for checking in while loop
        double lastReading = orientationSensor.getHeading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            double currentHeading = orientationSensor.getHeading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
            }
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < (stopEarly ? 10.0 : 0.5)) break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0) break;
            if (deltaD < 0 && currentHeading - finalHeading < 0) break;
            //timeout, break. default timeout: 3s
            if (System.currentTimeMillis() - iniTime > 3000) break;
            //stop pressed, break
            if (Thread.currentThread().isInterrupted()) break;
            lastReading = currentHeading;
            // yield handler
            this.core.yield();
        }
        stop();
        driveMode = DriveMode.STOP;
        useScalePower = true;
    }

    public void setRunMode(DcMotor.RunMode rm) {
        motorFL.setMode(rm);
        motorFR.setMode(rm);
        motorBL.setMode(rm);
        motorBR.setMode(rm);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        motorFL.setZeroPowerBehavior(zpb);
        motorFR.setZeroPowerBehavior(zpb);
        motorBL.setZeroPowerBehavior(zpb);
        motorBR.setZeroPowerBehavior(zpb);
    }

    public void stop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void reset() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        driveMode = DriveMode.STOP;
        targetHeading = 0;
    }


    public enum Direction {
        FRONT, LEFT, RIGHT, BACK;
    }

    public enum Wall {
        LEFT, RIGHT;
    }

    public double getCurHeading() {
        return curHeading;
    }

    /**
     * Scales power according to <code>minPower</code> and <code>maxPower</code> settings
     */
    private double scalePower(double power) {
        double adjustedPower = Math.signum(power) * minPower + power * (maxPower - minPower);
        return Math.abs(adjustedPower) > 1.0 ? Math.signum(adjustedPower) : adjustedPower;
    }

    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();
        line.addData("Pwr/Scale", new Func<String>() {
            @Override
            public String value() {
                return String.format("%.2f / %.1f", motorFL.getPower(), getDefaultScale());
            }
        });
        if (motorFL != null) {
            line.addData("FL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFL.getCurrentPosition();
                }
            });
        }
        if (motorFR != null) {
            line.addData("FR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorFR.getCurrentPosition();
                }
            });
        }
        if (motorBL != null) {
            line.addData("BL", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBL.getCurrentPosition();
                }
            });
        }
        if (motorBR != null) {
            line.addData("BR", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return motorBR.getCurrentPosition();
                }
            });
        }
        if (verticalLeftEncoder != null) {
            line.addData("verticalLeftEncoder=", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return verticalLeftEncoder.getCurrentPosition();
                }
            });
        }
        if (verticalRightEncoder != null) {
            line.addData("verticalRightEncoder=", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return verticalRightEncoder.getCurrentPosition();
                }
            });
        }
        if (horizontalEncoder != null) {
            line.addData("horizontalEncoder=", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return horizontalEncoder.getCurrentPosition();
                }
            });
        }


        //set up imu telemetry
        if (orientationSensor != null && setImuTelemetry) {
            line.addData("imuC", "%.1f", new Func<Double>() {
                @Override
                public Double value() {
                    return orientationSensor.getHeading();
                }
            });
            orientationSensor.setupTelemetry(line);
        }

        telemetry.addLine().addData("M", new Func<String>() {
            @Override
            public String value() {
                return driveMode.name();
            }
        }).addData("Head", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f (%+.2f)", targetHeading, headingDeviation);
            }
        }).addData("Adj", new Func<String>() {
            @Override
            public String value() {
                if (driveMode != DriveMode.STRAIGHT) return "N/A";
                return String.format("%+.1f", servoCorrection);
            }
        });
    }

    public void resetOrientation() {
        orientationSensor.reset();
    }

    public boolean hasRollStabalized(int inputIndex, double minDiff) {
        return orientationSensor.hasRollStabalized(inputIndex, minDiff);
    }
}
