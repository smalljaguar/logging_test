#pragma once
/* matrix library */
#include <BasicLinearAlgebra.h>

/*
Relevant variables:
Internal State
state vector x (calculated, best estimate of true state)
covariance matrix P (calculated, uncertainty of state)
each element of the matrix P_ij is the degree of correlation
between the ith state variable and the jth state variable

sensor conversion matrix H (constant)
sensor noise/covariance R
(derived from sensor datasheet)

State transition matrix F (constant)
(derived from the system dynamics e.g. kinematic equations)
State transition noise/covariance Q (can _assume_ constant)
No idea how to obtain, guess?

conversion mean z (measurement from sensor)

External influence
control matrix B (constant, derived from e.g. kinematic equations)
control vector u 
indicates the magnitude of any control system's or user's control on the situation


Kalman gain K (derived value)
can be interpreted as the size of an update
or how much to trust the sensor vs the model
*/

/*
final formulae:
predicting:
x = F*x + B*u
P = F*P*F^T + Q

updating:
K' = P*H^T*(H*P*H^T + R)^-1
x' = x + K*(z-H*x)
P' = P-K*H*P = (I-K*H)*P
*/