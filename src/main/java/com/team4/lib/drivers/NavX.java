package com.team4.lib.drivers;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.SPI;

/**
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class NavX implements IMU {
    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
                                            Object context) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (mLastSensorTimestampMs != kInvalidTimestamp && mLastSensorTimestampMs < sensor_timestamp) {
                    mYawRateDegreesPerSecond = 1000.0 * (-mYawDegrees - update.yaw)
                            / (double) (sensor_timestamp - mLastSensorTimestampMs);
                }
                mLastSensorTimestampMs = sensor_timestamp;
                mYawDegrees = -update.yaw;
                mFusedHeading = -update.fused_heading;
            }
        }
    }

    protected AHRS mAHRS;

    protected Rotation2d mAngleAdjustment = Rotation2d.identity();
    protected double mYawDegrees;
    protected double mFusedHeading;
    protected double mYawRateDegreesPerSecond;
    protected final long kInvalidTimestamp = -1;
    protected long mLastSensorTimestampMs;

    protected double mPrevAccelX = 0;
    protected double mPrevAccelY = 0;
    protected double mPrevTimeAccel = 0;

    public NavX() {
        this(SPI.Port.kMXP);
    }

    public NavX(SPI.Port spi_port_id) {
        mAHRS = new AHRS(spi_port_id, (byte) 200);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    @Override
    public boolean isPresent() {
        return mAHRS.isConnected();
    }

    @Override
    public synchronized boolean reset() {
        mAHRS.reset();
        resetState();
        return true;
    }

    public synchronized void zeroYaw() {
        mAHRS.zeroYaw();
        resetState();
    }

    private void resetState() {
        mLastSensorTimestampMs = kInvalidTimestamp;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation2d adjustment) {
        mAngleAdjustment = adjustment;
    }

    @Override
    public synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }

    public Rotation2d getYaw() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
    }

    public double getRoll() {
        return mAHRS.getRoll();
    }

    public double getPitch() {
        return mAHRS.getPitch();
    }

    public double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    @Override
    public double getFusedHeading() {
        return mFusedHeading;
    }

    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }

    public double getRawAccelX() {
        return mAHRS.getRawAccelX();
    }

    public double getRawAccelY() {
        return mAHRS.getRawAccelY();
    }

    public double getRawAccelZ() {
        return mAHRS.getRawAccelZ();
    }


    // public boolean isCollisionOccurring() {
    //     boolean collisionOccurring = false;

    //     double accelX = mAHRS.getWorldLinearAccelX();
    //     double accelY = mAHRS.getWorldLinearAccelY();


    //     double currTime = Timer.getFPGATimestamp();
    //     double dt = currTime-mPrevTimeAccel;

    //     double jerkX = (accelX - mPrevAccelX)/(dt);
    //     double jerkY = (accelY - mPrevAccelY)/(dt);

    //     mPrevAccelX = accelX;
    //     mPrevAccelY = accelY;

    //     if (mPrevTimeAccel == 0) {
    //         mPrevTimeAccel = currTime;
    //         return false;
    //     }

    //     mPrevTimeAccel = currTime;
    //     return collisionOccurring;
    // }

}