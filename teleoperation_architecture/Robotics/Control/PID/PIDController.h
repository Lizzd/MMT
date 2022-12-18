//
// Created by zican on 05.08.22.
//

#ifndef TELEOPERATION_ARCHITECTURE_AHTALKA_PIDCONTROLLER_H
#define TELEOPERATION_ARCHITECTURE_AHTALKA_PIDCONTROLLER_H

#include <vector>
#include <list>
#include <Eigen/Dense>

using namespace std;


struct PIDParameters {
    double k_p;
    double k_i;
    double k_d;

    PIDParameters() : k_p(0.0), k_i(0.0), k_d(0.0) {};

    PIDParameters(double p, double i, double d) : k_p(p), k_i(i), k_d(d) {};
};

//todo: as a manager class, this should be designed as a singleton
class PIDController {
private:
    PIDParameters myPIDParameters;
    list<Eigen::Vector3d> historyList;
    double history_sum_x = 0.0;
    double history_sum_y = 0.0;
    double history_sum_z = 0.0;
    size_t max_history_size = 100;

    double force_x_limit = 5;
    double force_y_limit = 5;
    double force_z_limit = 5;

public:
    PIDController() = default;

    PIDController(double p, double i, double d) : myPIDParameters(p, i, d) {};

    Eigen::Vector3d getPIDControlForce(const Eigen::Vector3d &targetPos, const Eigen::Vector3d &currentPos);

    Eigen::Vector3d getPDControlForce(const Eigen::Vector3d &targetPos, const Eigen::Vector3d &currentPos);

    Eigen::Vector3d getPDControlForceWithoutLimit(const Eigen::Vector3d &targetPos, const Eigen::Vector3d &currentPos);

    Eigen::Vector3d getPControlForce(const Eigen::Vector3d &targetPos, const Eigen::Vector3d &currentPos);

    inline void set_force_x_limit(double limit) { force_x_limit = limit; };

    inline void set_force_y_limit(double limit) { force_y_limit = limit; };

    inline void set_force_z_limit(double limit) { force_z_limit = limit; };

    void setPIDParameters(double _p, double _i, double _d) {
        myPIDParameters.k_p = _p;
        myPIDParameters.k_i = _i;
        myPIDParameters.k_d = _d;
    }

    Eigen::Vector3d getPIDParameters() const {
        return Eigen::Vector3d{myPIDParameters.k_p, myPIDParameters.k_i, myPIDParameters.k_d};
    }

private:
    Eigen::Vector3d getMeanAndUpdateHistory(const Eigen::Vector3d &currentError);

    inline void limit_force(Eigen::Vector3d &input);
};


#endif //TELEOPERATION_ARCHITECTURE_AHTALKA_PIDCONTROLLER_H
