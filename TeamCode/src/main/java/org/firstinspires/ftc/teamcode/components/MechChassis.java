package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;
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

    // the following 4 ratio values are used to normalize 4 wheel motors to the same speed
    // whenever changing a wheel motor, it must be calibrated again
    private double ratioFL = 1.0;
    private double ratioFR = 0.8847;
    private double ratioBL = 0.9348;
    private double ratioBR = 0.9315;

    private double left_ratio = 1.0; // slow down ratio for left wheels to go straight
    private double right_ratio = 1.0; // slow down ratio for right wheels to go straight
    private double front_ratio = 0.95; // slow down ratio for front wheels to go 90 degree
    private double back_ratio = 1.0; // slow down ratio for front wheels to go 90 degree


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

    private void configure_IMUs(Configuration configuration) {
        orientationSensor = new CombinedOrientationSensor().configureLogging(logTag + "-sensor", logLevel);
        orientationSensor.configure(configuration.getHardwareMap(), "imu", "imu2");
    }


    //odometry motors
    DcMotorEx verticalLeftEncoder;
    DcMotorEx verticalRightEncoder;
    DcMotorEx horizontalEncoder;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    final double ODO_COUNTS_PER_INCH = 307.699557;
    final double ODO_COUNTS_PER_CM = ODO_COUNTS_PER_INCH / 2.54;


    String rfName = "motorFR" , lfName = "motorFL";
    String rbName = "motorBR";
    String lbName = "motorBL";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    final double TICKS_PER_CM = 16.86;//number of encoder ticks per cm of driving

    private double chassisAligmentPower = 0.25;


    public void setGlobalPosUpdate(OdometryGlobalCoordinatePosition val) { globalPositionUpdate=val;}
    public OdometryGlobalCoordinatePosition globalPositionUpdate() { return globalPositionUpdate; }
    public double odo_count_per_inch() {return ODO_COUNTS_PER_INCH;}
    public double odo_count_per_cm() {return ODO_COUNTS_PER_CM;}
    public DcMotorEx verticalLeftEncoder(){ return verticalLeftEncoder; }
    public DcMotorEx verticalRightEncoder(){ return verticalRightEncoder; }
    public DcMotorEx horizontalEncoder(){ return horizontalEncoder; }

    public void configureOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeftEncoder(), verticalRightEncoder(), horizontalEncoder(), odo_count_per_inch(), 75);
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }

    public double odo_x_pos_inches() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnXCoordinate()/odo_count_per_inch();
    }
    public double odo_x_pos_cm() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnXCoordinate()/odo_count_per_cm();
    }


    public double odo_y_pos_inches() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnYCoordinate()/odo_count_per_inch();
    }
    public double odo_y_pos_cm() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnYCoordinate()/odo_count_per_cm();
    }

    public double odo_heading() {
        if (globalPositionUpdate==null) return 0;
        return globalPositionUpdate.returnOrientation();
    }

    public double getLeft_ratio() { return left_ratio; }
    public double getRight_ratio() { return left_ratio; }
    public double getFront_ratio() { return front_ratio; }
    public double getBack_ratio() { return back_ratio; }

    public double powerScale() { return powerScale; }

    public void enableRangeSensorTelemetry() { // must be call before reset() or setupTelemetry()
        setRangeSensorTelemetry = true;
    }

    public void enableImuTelemetry(Configuration configuration) {
        setImuTelemetry = true;
        if (orientationSensor==null) {
            configure_IMUs(configuration);
        }
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
            configure_IMUs(configuration);
        }

        // register chassis as configurable component
        configuration.register(this);
    }

    public void driveStraight(double power, double cm, double degrees, long timeout_sec){
        if (cm < 0){
            cm = - cm;
            power = - power;
        }
        boolean count_up = Math.signum(cm) > 0;

        double error_cm = 0.1 / 2.54;
        double target_y = cm + odo_y_pos_cm();
        long iniTime = System.currentTimeMillis();
        double cur_y = odo_y_pos_cm(), prev_y = cur_y;
        while(Math.abs(cur_y-target_y) > error_cm &&
                System.currentTimeMillis() - iniTime < timeout_sec * 1000){
                angleMove(degrees, power);
                if(count_up){
                    if (cur_y >= target_y - error_cm) break;
                } else{
                    if (cur_y <= target_y + error_cm) break;
                }
                prev_y = cur_y;
                cur_y = odo_y_pos_cm();
        }
        stop();
    }
    public void driveStraightNew(double power, double cm, double degree, double slowDownPercent, long timeout_sec){
        if (cm < 0){
            cm = - cm;
            power = - power;
        }
        double powerUsed = power;
        boolean count_up = Math.signum(cm) > 0;

        double error_cm = 0.1 / 2.54;
        double target_x = Math.cos(degree) * cm + odo_x_pos_cm();
        double target_y = Math.sin(degree) * cm + odo_y_pos_cm();


        long iniTime = System.currentTimeMillis();
        double cur_x = odo_x_pos_cm(), prev_x = cur_x;
        double cur_y = odo_y_pos_cm(), prev_y = cur_y;
        while(Math.abs(cur_y-target_y) > cm &&
                System.currentTimeMillis() - iniTime < timeout_sec * 1000){
            double traveledPercent = Math.abs(cur_y-target_y) / Math.abs(cm);
            if (traveledPercent > slowDownPercent) {
                double apower = Math.abs(power);

                double pow = .25 * minPower + .75 * apower;
                if (traveledPercent < .25 + .75 * slowDownPercent)
                    pow = .5 * minPower + .5 * apower;
                else if (traveledPercent < .5 + .5 * slowDownPercent)
                    pow = .75 * minPower + .25 * apower;
                else if (traveledPercent < .75 + .25 * slowDownPercent) pow = minPower;

                if (pow < minPower) pow = minPower;

                powerUsed = pow * Math.signum(power);
            }
            double desiredDegree = Math.atan2(target_x - cur_x, target_y - cur_y);

            //move
            angleMove(desiredDegree, powerUsed);


            if(count_up){
                if (cur_y >= target_y - error_cm) break;
            } else{
                if (cur_y <= target_y + error_cm) break;
            }
            prev_x = cur_x;
            cur_x = odo_x_pos_cm();
            prev_y = cur_y;
            cur_y = odo_y_pos_cm();
        }
        stop();
    }

    /**
     * move in the vertical direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void yMove(int sgn, double power) {
        motorFL.setPower(sgn * power * left_ratio * ratioFL);
        motorFR.setPower(sgn * power * right_ratio * ratioFR);
        motorBL.setPower(sgn * power * left_ratio * ratioBL);
        motorBR.setPower(sgn * power * right_ratio * ratioBR);
    }

    /**
     * move in the horizontal direction
     *
     * @param sgn   can be either +1 or -1
     * @param power must be in range [0,1]
     */
    public void xMove(int sgn, double power) {
        motorFL.setPower(sgn * power * front_ratio * ratioFL);
        motorFR.setPower(-sgn * power * front_ratio * ratioFR);
        motorBL.setPower(-sgn * power * back_ratio * ratioBL);
        motorBR.setPower(sgn * power * back_ratio * ratioBR);
    }
    public void angleMove(double directionAngle, double power){
        double[] motorPowers  = new double[4];
        motorPowers[0] = (Math.sin(Math.toRadians(directionAngle))+ Math.cos(Math.toRadians(directionAngle)));
        motorPowers[1] = (Math.cos(Math.toRadians(directionAngle))- Math.sin(Math.toRadians(directionAngle)));
        motorPowers[2] = motorPowers[1];
        motorPowers[3] = motorPowers[0];
        double max = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        motorFL.setPower(motorPowers[0] * power * ratioFL);
        motorFR.setPower(motorPowers[1] * power * ratioFR);
        motorBL.setPower(motorPowers[2] * power * ratioBL);
        motorBR.setPower(motorPowers[3] * power * ratioBR);
    }
    public void freeStyle(double fl, double fr, double bl, double br) {
        motorFL.setPower(fl* ratioFL);
        motorFR.setPower(fr* ratioFR);
        motorBL.setPower(bl* ratioBL);
        motorBR.setPower(br* ratioBR);
    }

    public void forward(double power, double inches, long timeout_sec) {
        // power should always be positive
        // inches > 0 forward
        //        < 0 backward

        power = Math.abs(power);
        boolean count_up = (Math.signum(inches)>0);
        double error_inches = 0.1;
        double target_y = inches+odo_y_pos_inches();
        long iniTime = System.currentTimeMillis();
        double cur_y = odo_y_pos_inches(), prev_y=cur_y;
        while ((Math.abs(cur_y-target_y) > error_inches) && (System.currentTimeMillis()-iniTime<timeout_sec*1000)) {
            yMove((int) Math.signum(inches), power);
            if (count_up) {
                if (cur_y>=target_y-error_inches) break;
            } else {
                if (cur_y<=target_y+error_inches) break;
            }
                prev_y=cur_y;
            cur_y = odo_y_pos_inches();
        }
        stop();
    }

    public void crab(double power, double inches, long timeout_sec) {
        // power should always be positive
        // inches > 0, crab right 90 degree
        //        < 0, crab left 90 degree
        power = Math.abs(power);

        boolean count_up = (Math.signum(inches)>0);

        double error_inches = 0.1;
        double target_x = inches+odo_x_pos_inches();
        long iniTime = System.currentTimeMillis();
        double cur_x = odo_x_pos_inches(), prev_x=cur_x;
        while ((Math.abs(cur_x-target_x) > error_inches) && (System.currentTimeMillis()-iniTime<timeout_sec*1000)) {
            xMove((int) Math.signum(inches), power);
            if (count_up) {
                if (cur_x>=target_x-error_inches) break;
            } else {
                if (cur_x<=target_x+error_inches) break;
            }
            prev_x=cur_x;
            cur_x = odo_x_pos_inches();
        }
        stop();
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
                return String.format("%.2f / %.1f\n", motorFL.getPower(), getDefaultScale());
            }
        });
        /*
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
        */
        if (horizontalEncoder != null) {
            line.addData("row X", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return horizontalEncoder.getCurrentPosition();
                }
            });
        }
        if (verticalLeftEncoder != null) {
            line.addData("row Y-Left", "%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return verticalLeftEncoder.getCurrentPosition();
                }
            });
        }
        if (verticalRightEncoder != null) {
            line.addData("row Y-Right", "%d\n", new Func<Integer>() {
                @Override
                public Integer value() {
                    return verticalRightEncoder.getCurrentPosition();
                }
            });
        }

        if (globalPositionUpdate!=null) {
            line.addData("Odo (x,ly,ry)", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%5.0f,%5.0f,%5.0f)\n", globalPositionUpdate.XEncoder(),
                            globalPositionUpdate.leftYEncoder(),
                            globalPositionUpdate.rightYEncoder());
                }
            });
            line.addData("Odo-pos (x,y,angle)", new Func<String>() {
                @Override
                public String value() {
                    return String.format("(%4.2f, %4.2f, %4.2f)\n", odo_x_pos_inches(), odo_y_pos_inches(), odo_heading());
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

    public void rawRotateTo(double power, double finalHeading, boolean stopEarly, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        debug("rotateT0(pwr: %.3f, finalHeading: %.1f)", power, finalHeading);
        double iniHeading = orientationSensor.getHeading();
        double deltaD = finalHeading - iniHeading;
        debug("iniHeading: %.1f, deltaD: %.1f)", iniHeading, deltaD);
        //do not turn if the heading is close enough the target
        if (Math.abs(deltaD) < 0.5)
            return;
        //resolve the issue with +-180 mark
        if (Math.abs(deltaD) > 180) {
            finalHeading = finalHeading + (deltaD > 0 ? -360 : +360);
            deltaD = 360 - Math.abs(deltaD);
            deltaD = -deltaD;
            debug("Adjusted finalHeading: %.1f, deltaD: %.1f)", finalHeading, deltaD);
        }
        //break on reaching the target
        if (Thread.interrupted()) return;
        //for (WheelAssembly wheel : wheels)
          //  wheel.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ensure the condition before calling rotate()
        //driveMode = DriveMode.STOP;
        useScalePower = false;
        //***** routine to start the wheels ******//
        turn((int)Math.signum(deltaD), power);
        //***** End routine to start the wheels ******//
        //record heading for checking in while loop
        double lastReading = orientationSensor.getHeading();
        long iniTime = System.currentTimeMillis();
        while (true) {
            if (Thread.interrupted()) return;
            double currentHeading = orientationSensor.getHeading();
            //we cross the +-180 mark if and only if the product below is a very negative number
            if ((currentHeading * lastReading < -100.0) || (Math.abs(currentHeading - lastReading) > 180.0)) {
                //deltaD>0 => cross the mark clockwise; deltaD<0 => cross the mark anticlockwise
                finalHeading = finalHeading + (deltaD > 0 ? -360.0 : +360.0);
                debug("Crossing180, finalHeading: %.1f, deltaD:%.1f)", finalHeading, deltaD);
            }
            debug("currentHeading: %.1f, finalHeading: %.1f)", currentHeading, finalHeading);
            //if within acceptable range, terminate
            if (Math.abs(finalHeading - currentHeading) < (stopEarly ? power * 10.0 : 0.5)) break;
            //if overshoot, terminate
            if (deltaD > 0 && currentHeading - finalHeading > 0) break;
            if (deltaD < 0 && currentHeading - finalHeading < 0) break;
            //timeout, break. default timeout: 3s
            if (System.currentTimeMillis() - iniTime > timeout) break;
            //stop pressed, break
            if (Thread.interrupted()) return;
            lastReading = currentHeading;
//            sleep(0);
            // yield handler
            //TaskManager.processTasks();
            this.core.yield();
        }
        if (Thread.interrupted()) return;
        //stop
        stop();

        useScalePower = true;
    }
    public void rotateToOld(double power, double finalHeading, int timeout) throws InterruptedException {
        if (Thread.interrupted()) return;
        if (power < 0.3) {//when power is small, use a flat power output
            rawRotateTo(power, finalHeading, true, timeout);
            return;
        }
        //when power is big, use a piecewise power output
        double iniHeading = orientationSensor.getHeading();
        double iniHeading_P = -iniHeading;
        double finalHeading_P = -finalHeading;
        double absDiff = min(abs(finalHeading - iniHeading_P), 180 - abs(finalHeading - iniHeading_P));
        double firstTarget;
        if (cos(iniHeading_P * degreeToRad) * sin(finalHeading_P * degreeToRad) - cos(finalHeading_P * degreeToRad) * sin(iniHeading_P * degreeToRad) >= 0) {//rotating ccw
            firstTarget = iniHeading - 0.8 * absDiff;
        } else {
            firstTarget = iniHeading + 0.8 * absDiff;
        }
        //make sure first target stay in [-180,+180] range
        if (firstTarget > 180) firstTarget -= 360;
        if (firstTarget < -180) firstTarget += 360;

        rawRotateTo(power, firstTarget, true, timeout);
        //sleep(100);
        if (Thread.interrupted()) return;
        rawRotateTo(chassisAligmentPower, finalHeading, false, 1000);
    }

    static final double degreeToRad = PI / 180;
    static final double radToDegree = 180 / PI;
}
