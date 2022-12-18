//
// Created by zican on 05.08.22.
//

#include "PIDController.h"

Eigen::Vector3d PIDController::getMeanAndUpdateHistory(const Eigen::Vector3d& currentError) {
    size_t curListSize=historyList.size();
    //if the list size has exceeded the up-limit, we have to abandon the oldest data first.
    if(curListSize>max_history_size){
        Eigen::Vector3d & oldestData=historyList.front();
        history_sum_x-=oldestData.x();
        history_sum_y-=oldestData.y();
        history_sum_z-=oldestData.z();
        historyList.pop_front();
    }
    history_sum_x+=currentError.x();
    history_sum_y+=currentError.y();
    history_sum_z+=currentError.z();
    historyList.push_back(currentError);
    return Eigen::Vector3d{history_sum_x,history_sum_y,history_sum_z};
}

Eigen::Vector3d PIDController::getPControlForce(const Eigen::Vector3d & targetPos,const Eigen::Vector3d & currentPos){
    Eigen::Vector3d ans=myPIDParameters.k_p*(targetPos-currentPos);
    limit_force(ans);
    return ans;
}

Eigen::Vector3d PIDController::getPDControlForceWithoutLimit(const Eigen::Vector3d &targetPos,
                                                                   const Eigen::Vector3d &currentPos) {
    Eigen::Vector3d lastError(0.0,0.0,0.0);
    if(!historyList.empty())
    {
        lastError=historyList.back();
    }
    //note that the last function is only p control

    return myPIDParameters.k_d*((targetPos-currentPos)-lastError)
           + getPControlForce(targetPos,currentPos);
}

Eigen::Vector3d PIDController::getPDControlForce(const Eigen::Vector3d & targetPos,const Eigen::Vector3d & currentPos){
    Eigen::Vector3d ans=getPDControlForceWithoutLimit(targetPos,currentPos);
    limit_force(ans);
    return ans;
}

Eigen::Vector3d PIDController::getPIDControlForce(const Eigen::Vector3d &targetPos, const Eigen::Vector3d &currentPos) {

    //note that the last function is only pd control
    Eigen::Vector3d ans=myPIDParameters.k_i* getMeanAndUpdateHistory(targetPos-currentPos)
                  + getPDControlForceWithoutLimit(targetPos,currentPos);
    limit_force(ans);
    return ans;
}

//this function is called frequently,so we prefer not to make function calls so often
void PIDController::limit_force(Eigen::Vector3d &input) {
    double x=input.x();
    double y=input.y();
    double z=input.z();
    if(abs(x)>force_x_limit){
        if(x<0)x=-force_x_limit;
        else x=force_x_limit;
    }
    if(abs(y)>force_y_limit){
        if(y<0)y=-force_y_limit;
        else y=force_y_limit;
    }
    if(abs(z)>force_z_limit){
        if(z<0)z=-force_z_limit;
        else z=force_z_limit;
    }
    input<<x,y,z;
}
