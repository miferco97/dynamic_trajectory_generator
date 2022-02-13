#ifndef __TRAJ_MODIFIERS_HPP__
#define __TRAJ_MODIFIERS_HPP__

#include "utils/logging_utils.hpp"

#include <cmath>
#include <exception>

#define SIGMA_FIXED 1.5
#define SIGMA_COEFFICIENT 0.75
#define MAX_REACTION_TIME 3

class TrajModifier
{
public:
  virtual void setDifference(const double &difference) = 0;
  virtual void setModifierTime(const double &delay_t) = 0;
  virtual double operator()(const double &t, int derivative) const = 0;
};

class GaussianModifier : public TrajModifier
{
  double sigma_ = SIGMA_FIXED;
  double sigma_squared_ = SIGMA_FIXED * SIGMA_FIXED;
  double mu_ = 0;
  double increment_ = 0;

public:
  GaussianModifier(){};

  void setModifierTime(const double &delay_t) override
  {
    mu_ = delay_t;
  };

  void setDifference(const double &difference) override
  {
    increment_ = difference;
  }
  inline void setSigma(const double &sigma)
  {
    sigma_ = sigma;
    sigma_squared_ = sigma * sigma;
  };
  inline double getSigma() { return sigma_; };

  double operator()(const double &t, int derivative = 0) const override
  {
    // double max_reaction_time = MAX_REACTION_TIME;
    double max_reaction_time = 3.5 * sigma_;
    if (std::abs(increment_) < 0.02 || std::abs(t - mu_) > max_reaction_time)
    {
      return 0;
    }
    double value = 0;
    switch (derivative)
    {
    case 0:
      value = std::exp(-1 * (t - mu_) * (t - mu_) / (2 * sigma_squared_));
      break;
    case 1:
      value = -((t - mu_) / (sigma_squared_)) * std::exp(-((t - mu_) * (t - mu_)) / (2 * sigma_squared_));
      break;
    case 2:
      value = ((t - mu_) / (sigma_squared_)) * ((t - mu_) / (sigma_squared_)) * std::exp(-((t - mu_) * (t - mu_)) / (2 * sigma_squared_));
      break;
    }
    value *= increment_;
    return value;
  }
};

#endif
