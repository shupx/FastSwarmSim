/**
 * @file quadrotor_dynamics.cpp
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

#include "fss_px4_sim/quadrotor_dynamics.hpp"

namespace odeint = boost::numeric::odeint;

namespace MavrosQuadSimulator
{

Dynamics::Dynamics()
{   /* Initialize */
    sim_step_ = 0.001; // seconds
    mass_ = 1.0; // kg
    g_ = 9.80665f; // m/s^2 geo.h CONSTANTS_ONE_G

    /* Initialize state */
    state_.pos = Eigen::Vector3d::Zero();
    state_.vel = Eigen::Vector3d::Zero();
    state_.R = Eigen::Matrix3d::Identity();
    vectorizeState(state_vec_, state_); // Initialize state_vec_
    acc_ = Eigen::Vector3d::Zero();
    q_ = Eigen::Quaterniond(1, 0, 0, 0);

    /* Initialize input */
    thrust_ = 0.0;
    omega_ = Eigen::Vector3d::Zero();
}

void Dynamics::differentialEquation(const StateVector& x, StateVector& dxdt, double t)
{
  State cur_state; // current state
  devectorizeState(cur_state, x); // devectorize x into cur_state

  State dot_state; // first order derivative of cur_state

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  // Get omega vee
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());
  omega_vee(2, 1) = omega_(0);
  omega_vee(1, 2) = -omega_(0);
  omega_vee(0, 2) = omega_(1);
  omega_vee(2, 0) = -omega_(1);
  omega_vee(1, 0) = omega_(2);
  omega_vee(0, 1) = -omega_(2);

  /* Differential Equations */
  dot_state.pos = cur_state.vel;
  dot_state.vel = -Eigen::Vector3d(0, 0, g_) + thrust_ * R.col(2) / mass_;
        //@TODO:  + external_force_ / mass_ - resistance * vnorm / mass_;
  dot_state.R = R * omega_vee;

  acc_ = dot_state.vel;
//   q_ = Eigen::Quaterniond(R);
//   q_.normalize();

  vectorizeState(dxdt, dot_state); // vectorize dot_state into dxdt

  // substitute nan value of dxdt with 0.0
  for (int i = 0; i < dxdt.size(); ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
      // std::cout << "nan apply to 0 for " << i << std::endl;
    }
  }
}

void Dynamics::step(const double& start_time, const double& end_time)
{
  vectorizeState(state_vec_, state_); // vectorize state_ into state_vec_ vector

  auto save = state_vec_; // save vector before integrate

  // integrate from start_time to end_time with step of sim_step_
  odeint::integrate(std::bind(&Dynamics::differentialEquation, this, std::placeholders::_1, 
                    std::placeholders::_2,std::placeholders::_3), state_vec_, start_time, end_time, sim_step_);

  // check nan value in state_vec_
  for (int i = 0; i < state_vec_.size(); ++i)
  {
    if (std::isnan(state_vec_[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      state_vec_ = save;
      break;
    }
  }

  devectorizeState(state_, state_vec_); // devectorize the updated state_vec_ into state_

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = state_.R * P.inverse();
  state_.R = R;

  //@TODO: simulate floor, wall and obstacles that can not be passed through.
  /* simulate the floor */
  if (state_.pos[2] < 0.0)
  {
    state_.pos[2] = 0.0;
    state_.vel[2] = 0.0;
    acc_[2] = 0.0;
  }
}

void Dynamics::setState(const Dynamics::State &state)
{
    state_.pos = state.pos;
    state_.vel = state.vel;
    state_.R   = state.R;
}

void Dynamics::setPos(const double& pos_x, const double& pos_y, const double& pos_z)
{
    state_.pos << pos_x, pos_y, pos_z;
}

void Dynamics::setRPY(const double& roll, const double& pitch, const double& yaw)
{
    using namespace Eigen;
    AngleAxisd rollAngle(AngleAxisd(roll,Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(pitch,Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));
    state_.R = yawAngle * pitchAngle * rollAngle; //@TODO make sure is RPY rather than YPR
}

void Dynamics::setInput(const Dynamics::Input &input)
{
    omega_ = input.omega; //@TODO the yaw rate setpoint can not be responsed immediately
    thrust_ = input.thrust;
}

void Dynamics::setSimStep(const double& dt)
{
    sim_step_ = dt;
}

void Dynamics::setMass(const double& m)
{
    mass_ = m;
}

void Dynamics::setGravityAcc(const double& g)
{
    g_ = g;
}

Dynamics::State Dynamics::getState()
{
    return state_;
}

double Dynamics::getSimStep()
{
    return sim_step_;
}

double Dynamics::getMass()
{
    return mass_;
}

double Dynamics::getGravityAcc()
{
    return g_;
}

Eigen::Vector3d Dynamics::getAcc()
{
    return acc_;
}

Eigen::Quaterniond Dynamics::getQuat()
{
  q_ = Eigen::Quaterniond(state_.R);
  q_.normalize();
  return q_;
}

Eigen::Vector3d Dynamics::getAngVel()
{
    return omega_;
}


void Dynamics::vectorizeState(StateVector& vec, const Dynamics::State& state)
{
    /* Vectorize 'state' into 'vec' */
    if (vec.empty()){
        /* Initialize state vecotr 'vec'*/
        for (int i = 0; i < 3; ++i)
            {vec.emplace_back(state.pos(i)); }
        for (int i = 0; i < 3; ++i)
            {vec.emplace_back(state.vel(i)); }
        for (int i = 0; i < 3; ++i)
            {vec.emplace_back(state.R(i, 0)); }
        for (int i = 0; i < 3; ++i)
            {vec.emplace_back(state.R(i, 1)); }
        for (int i = 0; i < 3; ++i)
            {vec.emplace_back(state.R(i, 2)); }
    }
    else{
        /* Update state vector 'vec'*/
        for (int i = 0; i < 3; ++i)
        {
            vec[0 + i]  = state.pos(i);
            vec[3 + i]  = state.vel(i);
            vec[6 + i]  = state.R(i, 0);
            vec[9 + i]  = state.R(i, 1);
            vec[12 + i] = state.R(i, 2);        
        }
    }
}

void Dynamics::devectorizeState(Dynamics::State& state, const StateVector& vec)
{
    /* Devectorize 'vec' into 'state' */
    for (int i = 0; i < 3; ++i)
    {
        state.pos(i) = vec[0 + i];
        state.vel(i) = vec[3 + i];
        state.R(i, 0) = vec[6 + i];
        state.R(i, 1) = vec[9 + i];
        state.R(i, 2) = vec[12 + i];        
    }
}

}
