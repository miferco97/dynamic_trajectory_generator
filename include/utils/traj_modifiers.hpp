#ifndef __TRAJ_MODIFIERS_HPP__
#define __TRAJ_MODIFIERS_HPP__

#include <cmath>
#define SIGMA_FIXED 1
#define MAX_REACTION_TIME 3

class TrajModifier
{
public:
  virtual void setDifference(const double &difference) = 0;
  virtual void setModifierTime(const double &delay_t) = 0;
  virtual double operator()(const double &t, const int &derivative) const = 0;
};

class GaussianModifier : public TrajModifier
{
  double sigma_ = SIGMA_FIXED;
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

  double operator()(const double &t, const int &derivative = 0) const override
  {
    if (std::abs(increment_) < 0.02 || std::abs(t - mu_) > MAX_REACTION_TIME)
    {
      return 0;
    }
    else
    {
      return increment_ * std::exp(-1 * (t - mu_) * (t - mu_) / (2 * sigma_ * sigma_));
    }
    {
      return 0;
    }
    {
      return 0;
    }
    double value = 0;
    switch (derivative)
    {
    case 0:
      value = std::exp(-((t - mu_) * (t - mu_)) / (2 * sigma_ * sigma_));
    case 1:
      value = -((t - mu_) / (sigma_ * sigma_)) * std::exp(-((t - mu_) * (t - mu_)) / (2 * sigma_ * sigma_));
    case 2:
      value = ((t - mu_) / (sigma_ * sigma_)) * ((t - mu_) / (sigma_ * sigma_)) * std::exp(-((t - mu_) * (t - mu_)) / (2 * sigma_ * sigma_));
    }
    value *= increment_;
    return value;
  }
};

#endif