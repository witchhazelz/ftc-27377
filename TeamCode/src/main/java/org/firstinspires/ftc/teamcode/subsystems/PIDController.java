import org.firstinspires.ftc.teamcode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.control.motion.Integrator;
import org.firstinspires.ftc.teamcode.control.filters.Filter;
import org.firstinspires.ftc.teamcode.control.filters.NoFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import com.qualcomm.robotcore.hardware.DcMotor;
public class PIDController {
    // PID components
    private PIDGains pidGains;
    private Integrator integrator;
    private Differentiator differentiator;
    private Filter filter;

    /**
     * constructor to initialize the PID controller with given gains.
     *
     * @param initialGains The initial PID gains.
     */
    public PIDController(PIDGains initialGains) {
        if (initialGains == null) {
            throw new IllegalArgumentException("PIDGains cannot be null");
        }
        this.pidGains = initialGains;
        this.integrator = new Integrator();
        this.differentiator = new Differentiator();
        this.filter = new NoFilter(); //  can be replaced with custom filters
    }

    /**
     * calculate PID output based on target and current values.
     *
     * @param target The desired target value.
     * @param current The current sensor/feedback value.
     * @return The calculated PID output.
     */
    public double calculateOutput(double target, double current) {
        double error = target - current;

        // proportional term
        double pTerm = pidGains.getKp() * error;

        // integral term
        double iTerm = pidGains.getKi() * integrator.update(error);

        // derivative term
        double dTerm = pidGains.getKd() * differentiator.update(error);

        // combine terms and return filtered output
        return filter.update(pTerm + iTerm + dTerm);
    }

    /**
     * sets new PID gains for live tuning (i think hopefully)
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     */
    public void setPIDGains(double kp, double ki, double kd) {
        pidGains.setKp(kp);
        pidGains.setKi(ki);
        pidGains.setKd(kd);
    }

    /**
     * Applies the PID output to a given motor.
     *
     * @param motor The motor to control.
     * @param targetPosition The target position for the motor.
     * @param currentPosition The current position of the motor.
     */
    public void applyToMotor(DcMotor motor, double targetPosition, double currentPosition) {
        if (motor == null) {
            throw new IllegalArgumentException("Motor cannot be null");
        }

        double output = calculateOutput(targetPosition, currentPosition);

        // makes sure output is within the motor power range (-1 to 1)
        output = Math.max(-1.0, Math.min(1.0, output));

        motor.setPower(output);
    }

    /**
     * retrieve current PID gains
     *
     * @return current PID gains
     */
    public PIDGains getPIDGains() {
        return new PIDGains(pidGains.getKp(), pidGains.getKi(), pidGains.getKd());
    }
}
