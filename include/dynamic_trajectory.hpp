#ifndef __DYNAMIC_TRAJECTORY_HPP__
#define __DYNAMIC_TRAJECTORY_HPP__

#include <vector>
#include <memory>
#include <iostream>

#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"

#include "matplotlibcpp.h"

#define SPEED 0.5


class DynamicTrajectory {
public:
    DynamicTrajectory(){};
private:
    std::shared_ptr<mav_trajectory_generation::Trajectory> traj_ptr_ = nullptr;
    const int dimension_ = 3;
    const double a_max_ = 1*9.81f;

public:
    std::shared_ptr<mav_trajectory_generation::Trajectory> generateNewTrajectory(const mav_trajectory_generation::Vertex::Vector& vertices, bool lineal_optimization = false){        
        // TODO: CHECK THE USE OF FUTUREs HERE for ASYNC calls
            // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::JERK;
        const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::ACCELERATION;
        // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

        double v_max = SPEED; // TODO: CHECK THIS VALUE

        auto segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max, a_max_);
        const int N = 10;
        mav_trajectory_generation::Segment::Vector segments;
        std::shared_ptr<mav_trajectory_generation::Trajectory> trajectory = std::make_shared<mav_trajectory_generation::Trajectory>();

        // Optimizer
        if (lineal_optimization)
        {
            mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
            opt.solveLinear();
            opt.getSegments(&segments);
            opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
        }

        else
        {
            mav_trajectory_generation::NonlinearOptimizationParameters parameters;
            parameters.max_iterations = 2000;
            parameters.f_rel = 0.05;
            parameters.x_rel = 0.1;
            parameters.time_penalty = 200.0;
            parameters.initial_stepsize_rel = 0.1;
            // parameters.inequality_constraint_tolerance = 0.1;
            parameters.inequality_constraint_tolerance = 0.2;

            mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension_, parameters);
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
            opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
            opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
            opt.optimize();
            opt.getPolynomialOptimizationRef().getSegments(&segments);
            opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
        }
        traj_ptr_ = trajectory;
        return trajectory;

    };

    void printTrajectory(){
        double t_start = traj_ptr_->getMinTime();
        double t_end = traj_ptr_->getMaxTime();
        double dt = 0.1;
        for (double t_eval = t_start; t_eval<t_end; t_eval+=dt){
            Eigen::VectorXd x_eval;
            x_eval = traj_ptr_->evaluate(t_eval, mav_trajectory_generation::derivative_order::POSITION);
            std::cout << "t: " << t_eval << " x: " << x_eval.transpose() << std::endl;
        }
    }

    void generate2Dplot(){
        namespace plt = matplotlibcpp;
        double t_start = traj_ptr_->getMinTime();
        double t_end = traj_ptr_->getMaxTime();
        double dt = 0.1;
        int n_samples = (t_end-t_start)/dt;

        plt::figure();        
        std::vector<double> x_vec(n_samples), y_vec(n_samples), z_vec(n_samples);
        std::vector<double> time(n_samples);

        for (int i=0; i<n_samples; i++){
            double t_eval = t_start + i*dt;
            Eigen::VectorXd x_eval;
            x_eval = traj_ptr_->evaluate(t_eval, mav_trajectory_generation::derivative_order::POSITION);
            x_vec[i] = x_eval(0);
            y_vec[i] = x_eval(1);
            z_vec[i] = x_eval(2);
            time[i] = t_eval;	
        }
        plt::plot(time, x_vec, "r-", time, y_vec, "g-", time, z_vec, "b-");
        plt::show(true);

    }

    void generate3DPlot(){

        namespace plt = matplotlibcpp;
        double t_start = traj_ptr_->getMinTime();
        double t_end = traj_ptr_->getMaxTime();
        double dt = 0.1;
        int n_samples = (t_end-t_start)/dt;

        plt::figure();
        std::vector<double> x_vec(n_samples), y_vec(n_samples), z_vec(n_samples);
        std::vector<double> time(n_samples);
        for (int i=0; i<n_samples; i++){
            double t_eval = t_start + i*dt;
            Eigen::VectorXd x_eval;
            x_eval = traj_ptr_->evaluate(t_eval, mav_trajectory_generation::derivative_order::POSITION);
            x_vec[i] = x_eval(0);
            y_vec[i] = x_eval(1);
            z_vec[i] = x_eval(2);
            time[i] = t_eval;	
        }
        plt::plot3(x_vec, y_vec, z_vec);
        plt::show(true);

    }

    void showPlots(){
        namespace plt = matplotlibcpp;
        // plt::show();
    }

};

#endif // __DYNAMIC_TRAJECTORY_HPP__