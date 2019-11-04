#ifndef KALMAN_H_INCLUDED
#define KALMAN_H_INCLUDED
typedef struct Kalman{
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};



void initKalman(struct Kalman* filter);
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(struct Kalman* filter, float newAngle, float newRate, float dt);

void setAngle(struct Kalman* filter,float angle); // Used to set angle, this should be set as the starting angle
float getRate(struct Kalman* filter); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
void setQangle(struct Kalman* filter,float Q_angle);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp.
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
void setQbias(struct Kalman* filter,float Q_bias);
void setRmeasure(struct Kalman* filter,float R_measure);

float getQangle(struct Kalman* filter);
float getQbias(struct Kalman* filter);
float getRmeasure(struct Kalman* filter);

#endif // KALMAN_H_INCLUDED
