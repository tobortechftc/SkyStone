package org.firstinspires.ftc.teamcode.hardware.Sigma;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.AdjustableServo;
import org.firstinspires.ftc.teamcode.components.CombinedOrientationSensor;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.Progress;
import org.firstinspires.ftc.teamcode.support.tasks.Task;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

/**
 * StoneGrabber spec:
 */
public class StoneGrabber extends Logger<StoneGrabber> implements Configurable {

    final private CoreSystem core;

    private DcMotor lifter;
    private AdjustableServo arm;
    private AdjustableServo wrist;
    private AdjustableServo grabber;

    private final double ARM_UP = 0.1;
    private final double ARM_DOWN = 0.9;
    private final double ARM_INITIAL = 0.9;
    private final double ARM_OUT = 0.4;
    private final double ARM_DELIVER = 0.3;

    private final double WRIST_INIT = 0.5;
    private final double WRIST_PARALLEL = 0.151;
    private final double WRIST_PERPENDICULAR = 0.5;

    private final double GRABBER_INIT = 0.5;
    private final double GRABBER_OPEN = 0.143;
    private final double GRABBER_CLOSE = 0.55;

    private final int LIFT_DOWN = 50;
    private final int LIFT_MAX = 1000;
    private final int LIFT_SAFE_SWING = 250;
    private final double LIFT_POWER = 0.5;


    private boolean armIsDown = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public String getUniqueName() {
        return "StoneGrabber";
    }

    @Override
    public void setAdjustmentMode(boolean on) {
        // this method does nothing since chassis has no hardware
        //  that would react to track / wheel base / radius adjustments
    }

    /**
     * Hanging constructor
     */
    public StoneGrabber(CoreSystem core) {
        this.core = core;
    }

    public void reset(boolean Auto) {
        if (arm != null)
            armInit();
        if (wrist!=null)
            wristInit();
        if (grabber!=null)
            grabberInit();
    }

    public void configure(Configuration configuration, boolean auto) {
        arm = new AdjustableServo(0,1).configureLogging(
                logTag + ":arm", logLevel
        );
        arm.configure(configuration.getHardwareMap(), "arm");
        configuration.register(arm);
        armInit();

        wrist = new AdjustableServo(0,1).configureLogging(
                logTag + ":wrist", logLevel
        );
        wrist.configure(configuration.getHardwareMap(), "wrist");
        configuration.register(wrist);
        wristInit();

        grabber = new AdjustableServo(0,1).configureLogging(
                logTag + ":grabber", logLevel
        );
        grabber.configure(configuration.getHardwareMap(), "grabber");
        configuration.register(grabber);
        grabberInit();

        lifter = configuration.getHardwareMap().tryGet(DcMotor.class, "lifter");
        if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // register hanging as configurable component
        // configuration.register(this);
    }

    public void armInit() {
        arm.setPosition(ARM_INITIAL);
        armIsDown = false;
    }

    public void armOut() {
        arm.setPosition(ARM_OUT);
        armIsDown = false;
    }

    public void armUp() {
        arm.setPosition(ARM_UP);
        armIsDown = false;
    }

    public void armDown() {
        arm.setPosition(ARM_DOWN);
        armIsDown = true;
    }

    public void armDeliver() {
        arm.setPosition(ARM_DELIVER);
    }

    public void armAuto() {
        if (armIsDown) {
            armUp();
        } else {
            armDown();
        }
    }

    public void wristInit() {
        if (wrist==null) return;
        wrist.setPosition(WRIST_INIT);
    }

    public void wristParallel () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PARALLEL);
    }

    public void wristPerpendicular () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PERPENDICULAR);
    }

    public void grabberInit() {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_INIT);
    }

    public void grabberOpen () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_OPEN);
    }

    public void grabberClose () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_CLOSE);
    }

    public void liftUp () {
        if (lifter==null) return;
        lifter.setPower(LIFT_POWER);
    }

    public void liftDown() {
        if (lifter==null) return;
        lifter.setPower(-LIFT_POWER);
    }

    public void liftStop() {
        if (lifter==null) return;
        lifter.setPower(0);
    }

    public Progress liftToPosition(int pos) {
        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(pos);
        lifter.setPower(LIFT_POWER);
        return new Progress() {
            public boolean isDone() {
                return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 20;
            }
        };
    }

    private Progress moveArm(double position) {
        double adjustment = Math.abs(position - arm.getPosition());
        arm.setPosition(position);
        // 3.3ms per degree of rotation
        final long doneBy = System.currentTimeMillis() + Math.round(adjustment * 900);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void armOutCombo() {
        final String taskName = "deliveryCombo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen();
                liftToPosition(LIFT_SAFE_SWING);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 20;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_OUT);
            }
        }, taskName);
        liftToPosition(LIFT_DOWN);
    }

    public void armInCombo() {

    }

    public void grabStoneCombo() {

    }

    public void deliverStoneCombo() {

    }
    /**
     * Set up telemetry lines for chassis metrics
     * Shows current motor power, orientation sensors,
     * drive mode, heading deviation / servo adjustment (in <code>STRAIGHT</code> mode)
     * and servo position for each wheel
     */
    public void setupTelemetry(Telemetry telemetry) {
        Telemetry.Line line = telemetry.addLine();

        if (arm != null) {
            line.addData("arm", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return arm.getPosition();
                }
            });
        }
        if (wrist != null) {
            line.addData("wrist", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return wrist.getPosition();
                }
            });
        }
        if (grabber != null) {
            line.addData("grabber", "pos=%.2f", new Func<Double>() {
                @Override
                public Double value() {
                    return grabber.getPosition();
                }
            });
        }
        if (lifter != null) {
            line.addData("lifter", "pos=%d", new Func<Integer>() {
                @Override
                public Integer value() {
                    return lifter.getCurrentPosition();
                }
            });
        }

    }

}



