package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFControllerCTS
{
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    private double setpoint;
    private double previousError;
    private double integralSum;

    private double tolerance = 0;

    private double minOutput;
    private double maxOutput;

    private final ElapsedTime timer = new ElapsedTime();

    // Guard against an enormous dt on the very first loop (or after a reset)
    // which would blow up the derivative term.
    private static final double MAX_DELTA_TIME = 0.5;  // seconds
    private static final double MIN_DELTA_TIME = 1e-6; // prevent divide-by-zero

    /**
     * Constructor for the PIDFController.
     *
     * @param Kp        Proportional gain
     * @param Ki        Integral gain
     * @param Kd        Derivative gain
     * @param Kf        Feed-forward gain
     * @param minOutput Minimum output value
     * @param maxOutput Maximum output value
     */
    public PIDFControllerCTS(double Kp, double Ki, double Kd, double Kf,
                          double minOutput, double maxOutput)
    {
        this.Kp        = Kp;
        this.Ki        = Ki;
        this.Kd        = Kd;
        this.Kf        = Kf;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        reset();
    }

    /**
     * Sets the desired setpoint for the controller.
     *
     * @param setpoint The target value
     */
    public void setTarget(double setpoint)
    {
        this.setpoint = setpoint;
    }

    /**
     * Calculates the control output based on the current process variable.
     *
     * FIX 1 — Derivative is now computed on the ERROR DELTA, not the raw error
     * vs previousError divided by a timer that may have just been reset. This
     * eliminates derivative kick from setpoint changes and near-zero dt spikes.
     *
     * FIX 2 — The timer no longer resets inside the tolerance check. Previously
     * the timer was reset every time the process variable was inside tolerance,
     * so on the next loop deltaTime was near-zero and the derivative term
     * exploded — kicking the robot back into oscillation.
     *
     * FIX 3 — Integral anti-windup: the integral only accumulates when the
     * output is not saturated (i.e. not already hitting the min/max limit).
     * This prevents the integralSum from growing unbounded and causing overshoot.
     *
     * @param processVariable The current measured value
     * @return The calculated control output
     */
    public double calculate(double processVariable)
    {
        double deltaTime = timer.seconds();
        timer.reset();  // Always reset — timer is now purely for dt measurement,
        // not gated behind the tolerance check (FIX 2).

        // Clamp deltaTime so a stale first reading or a paused loop can't
        // produce a garbage derivative spike.
        deltaTime = Math.max(MIN_DELTA_TIME, Math.min(MAX_DELTA_TIME, deltaTime));

        double error = setpoint - processVariable;

        // --- Tolerance dead-band ---
        // Return 0 but keep previousError updated so the derivative is smooth
        // when the robot leaves the tolerance band again.
        if (Math.abs(error) <= tolerance)
        {
            previousError = error;  // don't freeze previousError at an old value
            integralSum   = 0;      // bleed the integral while we're on target
            return 0;
        }

        // --- Proportional ---
        double proportionalTerm = Kp * error;

        // --- Integral with anti-windup (FIX 3) ---
        // Speculatively accumulate, then undo if output would be saturated.
        double prospectiveIntegral = integralSum + error * deltaTime;
        double integralTerm        = Ki * prospectiveIntegral;

        // --- Derivative on error delta (FIX 1) ---
        // Use (error - previousError) / dt rather than raw error / dt.
        // This means Kd damps the *rate of change of error*, not the error itself.
        double errorDelta      = error - previousError;
        double derivativeTerm  = Kd * (errorDelta / deltaTime);

        // --- Feed-forward ---
        double feedforwardTerm = Kf * setpoint;

        // --- Total output ---
        double output = proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;

        // --- Anti-windup: only commit the integral if output is not saturated ---
        boolean saturated = (output > maxOutput && error > 0)
                || (output < minOutput && error < 0);
        if (!saturated)
        {
            integralSum = prospectiveIntegral;
        }
        // If saturated, integralSum is NOT updated — prevents wind-up.

        // --- Clamp output ---
        output = Math.max(minOutput, Math.min(maxOutput, output));

        // --- Store error for next loop ---
        previousError = error;

        return output;
    }

    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        Kf = f;
    }

    /**
     * Resets the controller state. Call this when switching targets or
     * toggling the controller on/off to prevent stale integral and derivative
     * values from causing a startup kick.
     */
    public void reset()
    {
        integralSum   = 0;
        previousError = 0;
        timer.reset();
    }
}