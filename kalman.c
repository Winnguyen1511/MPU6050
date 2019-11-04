#include "kalman.h"

void initKalman(struct Kalman* filter)
{
    /* We will set the variables like so, these can also be tuned by the user */
    filter->Q_angle = 0.001f;
    filter->Q_bias = 0.003f;
    filter->R_measure = 0.03f;

    filter->angle = 0.0f; // Reset the angle
    filter->bias = 0.0f; // Reset bias

    filter->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 0.0f;
}


float getAngle(struct Kalman* filter,float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    filter->rate = newRate - filter->bias;
    filter->angle += dt * filter->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    filter->P[0][0] += dt * (dt*filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
    filter->P[0][1] -= dt * filter->P[1][1];
    filter->P[1][0] -= dt * filter->P[1][1];
    filter->P[1][1] += filter->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = filter->P[0][0] + filter->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = filter->P[0][0] / S;
    K[1] = filter->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - filter->angle; // Angle difference
    /* Step 6 */
    filter->angle += K[0] * y;
    filter->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = filter->P[0][0];
    float P01_temp = filter->P[0][1];

    filter->P[0][0] -= K[0] * P00_temp;
    filter->P[0][1] -= K[0] * P01_temp;
    filter->P[1][0] -= K[1] * P00_temp;
    filter->P[1][1] -= K[1] * P01_temp;

    return filter->angle;
}

void setAngle(struct Kalman* filter,float angle) { filter->angle = angle; } // Used to set angle, this should be set as the starting angle
float getRate(struct Kalman* filter) { return filter->rate; } // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(struct Kalman* filter,float Q_angle) { filter->Q_angle = Q_angle; }
void setQbias(struct Kalman* filter,float Q_bias) { filter->Q_bias = Q_bias; }
void setRmeasure(struct Kalman* filter,float R_measure) { filter->R_measure = R_measure; }

float getQangle(struct Kalman* filter) { return filter->Q_angle; }
float getQbias(struct Kalman* filter) { return filter->Q_bias; }
float getRmeasure(struct Kalman* filter) { return filter->R_measure; }
