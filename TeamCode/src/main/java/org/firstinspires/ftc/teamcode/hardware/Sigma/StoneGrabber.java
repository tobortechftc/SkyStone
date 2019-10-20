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
    private final double ARM_LOW = 0.6;
    private final double ARM_OUT = 0.45;
    private final double ARM_DELIVER = 0.3;
    private final double ARM_INC_UNIT = 0.02;

    private final double WRIST_PARALLEL = 0.18;
    private final double WRIST_PERPENDICULAR = 0.72;
    private final double WRIST_INIT = WRIST_PERPENDICULAR;

    private final double GRABBER_INIT = 0.31;
    private final double GRABBER_OPEN = 0.89;
    private final double GRABBER_CLOSE = 0.387;

    private final int LIFT_DOWN = 20;
    private final int LIFT_GRAB = 400;
    private final int LIFT_MAX = 3640;
    private final int LIFT_SAFE_SWING = 790;
    private final double LIFT_POWER = 0.5;
    private final int LIFT_DELIVER = 1000;


    private boolean armIsDown = false;
    private boolean isGrabberOpened = false;
    private boolean isWristParallel = false;
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
        if (lifter != null) lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void armDownInc() {
        double cur_pos = arm.getPosition();
        cur_pos += ARM_INC_UNIT;
        if (cur_pos>1) cur_pos=1;
        arm.setPosition(cur_pos);
        armIsDown = false;
    }
    public void armUpInc() {
        double cur_pos = arm.getPosition();
        cur_pos -= ARM_INC_UNIT;
        if (cur_pos<0) cur_pos=0;
        arm.setPosition(cur_pos);
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
        isWristParallel = false;
    }

    public void wristParallel () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PARALLEL);
        isWristParallel = true;
    }

    public void wristPerpendicular () {
        if (wrist==null) return;
        wrist.setPosition(WRIST_PERPENDICULAR);
        isWristParallel = false;
    }

    public void wristAuto() {
        if (isWristParallel) wristPerpendicular();
        else wristParallel();
    }

    public Progress moveWrist(boolean parallel) {
        double target = parallel ? WRIST_PARALLEL : WRIST_PERPENDICULAR;
        isWristParallel = parallel;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveGrabber(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from parallel to vertical takes 2 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(1200 * adjustment);
        wrist.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void grabberInit() {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_INIT);
    }

    public void grabberOpen () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_OPEN);
        isGrabberOpened = true;
    }

    public void grabberClose () {
        if (grabber==null) return;
        grabber.setPosition(GRABBER_CLOSE);
        isGrabberOpened = false;
    }

    public void grabberAuto() {
        if (isGrabberOpened)
            grabberClose();
        else
            grabberOpen();
    }

    public Progress moveGrabber(boolean closed) {
        double target = closed ? GRABBER_CLOSE : GRABBER_OPEN;
        isGrabberOpened = !closed;
        double adjustment = Math.abs(grabber.getPosition() - target);
        debug("moveGrabber(): target=%.2f, adjustment=%.2f", target, adjustment);
        // entire move from up to down takes 1 seconds
        final long doneBy = System.currentTimeMillis() + Math.round(600 * adjustment);
        grabber.setPosition(target);
        return new Progress() {
            @Override
            public boolean isDone() {
                return System.currentTimeMillis() >= doneBy;
            }
        };
    }

    public void liftUp () {
        if (lifter==null) return;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setPower(LIFT_POWER);
    }

    public void liftDown() {
        if (lifter==null) return;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setPower(-LIFT_POWER);
    }

    public void liftStop() {
        if (lifter==null) return;
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setPower(0);
    }

    public void liftToSafe() {
        liftToPosition(LIFT_SAFE_SWING);
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

    public void armOutCombo(final boolean shouldOpenGrabber) {
        final String taskName = "Arm Out Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                if (shouldOpenGrabber) grabberOpen();
                if (arm.getPosition()>ARM_LOW) // arm inside the robot
                    liftToPosition(LIFT_SAFE_SWING);
                else
                    liftToPosition(LIFT_GRAB);

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
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_GRAB);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 20;
                    }
                };
            }
        }, taskName);
    }

    public void armInCombo(final boolean wristParallel) {
        final String taskName = "Arm In Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task () {
            @Override
            public Progress start() {
                liftToPosition(LIFT_SAFE_SWING);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 20;
                    }
                };
            }
        }, taskName);
        TaskManager.add(new Task () {
            @Override
            public Progress start() {
                moveWrist(wristParallel);
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
                return moveArm(ARM_INITIAL);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 20;
                    }
                };

            }
        }, taskName);
    }


    public void grabStoneCombo() {
        final String taskName = "Grab Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;

        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 100;
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
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                final Progress grabberProgress = moveGrabber(true);
                return new Progress() {
                    @Override
                    public boolean isDone() { return grabberProgress.isDone();}
                };
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberClose();
                arm.setPosition(ARM_DELIVER);
                liftToPosition(LIFT_DOWN);
                return new Progress() {
                    @Override
                    public boolean isDone() { return !lifter.isBusy() || Math.abs(lifter.getTargetPosition() - lifter.getCurrentPosition()) < 20;
                    }
                };
            }
        }, taskName);

    }

    public void deliverStoneCombo() {
        final String taskName = "Deliver Stone Combo";
        if (!TaskManager.isComplete(taskName)) return;
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                return moveArm(ARM_OUT);
            }
        }, taskName);
        TaskManager.add(new Task() {
            @Override
            public Progress start() {
                grabberOpen(); return liftToPosition(LIFT_SAFE_SWING);
            }
        }, taskName);
    }
    /*
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



