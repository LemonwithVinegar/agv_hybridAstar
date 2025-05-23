//
// Created by abang on 17-10-26.
//

#ifndef UKF_UKF_H
#define UKF_UKF_H


#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <custom_msgs/Object.h>
#include <custom_msgs/ObjectArray.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    float flag = 0;
    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;
    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    MatrixXd H_laser_;

    MatrixXd R_laser_;

    MatrixXd R_radar_;

    ///* time when the state is true, in us
    // long long time_s_;
    double time_s_;
    // long long time_sum_;
    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_ax_;

    double std_ay_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    double std_lasvx_;

    double std_lasvy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;


    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    // void ProcessMeasurement(const custom_msgs::Object &object);
    void ProcessMeasurement(const custom_msgs::Object &object, const double &current_time);
    // void ProcessMeasurement(const custom_msgs::Object &object, const int &id, const double &current_time);
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(const custom_msgs::Object &object);
    // void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    // void UpdateRadar(MeasurementPackage meas_package);

    void AugmentedSigmaPoints(MatrixXd *Xsig_out);

    void SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t);

    void PredictMeanAndCovariance();

    void PredictRadarMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, long n_z);

    void UpdateState(VectorXd &z, VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, long n_z);

    void PredictLaserMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, long n_z);
};


#endif //UKF_UKF_H
