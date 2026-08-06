/**
 * @file quadrotor_dynamics.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Ordinary Differential Equations of quadrotor dynamics 
 * solved by odeint.
 * 
 * Note: This program relies on <boost/numeric/odeint.hpp>
 * 
 * @version 1.0
 * @date 2023-11-27
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __QUADROTOR_DYNAMICS_H__
#define __QUADROTOR_DYNAMICS_H__

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace MavrosQuadSimulator
{

class Dynamics
{
  public:
    // For internal use, but needs to be public for odeint
    typedef std::vector<double> StateVector;    

    struct State
    {
      Eigen::Vector3d pos;
      Eigen::Vector3d vel;
      Eigen::Matrix3d R;
      // Eigen::Vector3d omega;
      // Eigen::Array4d  motor_rpm;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    
    struct Input
    {
      Eigen::Vector3d omega;
      double thrust;
    };

    Dynamics();

    void setState(const Dynamics::State &state);
    void setPos(const double& pos_x, const double& pos_y, const double& pos_z);
    void setRPY(const double& roll, const double& pitch, const double& yaw);
    void setInput(const Dynamics::Input &input);
    void setSimStep(const double& dt);
    void setMass(const double& m);
    void setGravityAcc(const double& g);

    Dynamics::State getState();
    double getSimStep();
    double getMass();
    double getGravityAcc();
    Eigen::Vector3d getAcc();
    Eigen::Quaterniond getQuat();
    Eigen::Vector3d getAngVel();

    void differentialEquation(const StateVector& x, StateVector& dxdt, double t);

    // Run the actual dynamics during [start_time, end_time] with a time step of sim_step_
    void step(const double& start_time, const double& end_time);

  private:
    double sim_step_;
    double mass_;
    double g_;
    double thrust_;
    Eigen::Vector3d omega_;
    Eigen::Vector3d acc_;
    Eigen::Quaterniond q_;

    Dynamics::State state_;
    StateVector state_vec_;
    void vectorizeState(StateVector& vec, const Dynamics::State& state);
    void devectorizeState(Dynamics::State& state, const StateVector& vec);

};


}

#endif
