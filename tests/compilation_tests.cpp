
#include "dynamic_trajectory.hpp"
#include <iostream>

int main (){
    std::cout << "Hello World" << std::endl;
    DynamicTrajectory traj;

    mav_trajectory_generation::Vertex::Vector vertices(5,3);

    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::POSITION , Eigen::Vector3d(0, 0, 0));
    vertices[1].addConstraint(mav_trajectory_generation::derivative_order::POSITION , Eigen::Vector3d(-1, 1, 1));
    vertices[2].addConstraint(mav_trajectory_generation::derivative_order::POSITION , Eigen::Vector3d(2, -2, 2));
    vertices[3].addConstraint(mav_trajectory_generation::derivative_order::POSITION , Eigen::Vector3d(4, -3, 4));
    vertices[4].addConstraint(mav_trajectory_generation::derivative_order::POSITION , Eigen::Vector3d(5, 7, 2));

    traj.generateNewTrajectory(vertices, 1.0f);
    traj.plotTrajectory();
}