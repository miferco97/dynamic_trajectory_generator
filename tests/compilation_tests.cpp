#define __SCREEN_OUTPUT__

#include "dynamic_trajectory.hpp"
#include "traj_evaluator.hpp"
#include "utils/plotting_utils.hpp"
#include <iostream>

int main()
{
    using namespace dynamic_traj_generator;
    DynamicTrajectory traj;

    mav_trajectory_generation::Vertex::Vector vertices(5, 3);

    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 0));
    vertices[1].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1, 1, 1));
    vertices[2].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2, -2, 2));
    vertices[3].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4, -3, 4));
    vertices[4].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 7, 2));

    traj.generateTrajectoryFromScratch(vertices, 1.0f);
    TrajEvaluator eval;
    eval.runEvaluation(traj);
    // traj.generate2Dplot();
    // traj.generate3DPlot();
    // traj.showPlots();
}