#ifndef SLAM_LIBRARIES_H
#define SLAM_LIBRARIES_H
// SLAM libraries
#include "use-ikfom.hpp"
#include "ikd_Tree.h"
#endif

class Localizator {
    
    // Variables

    public:
        Points points2match;
        double last_time_integrated = -1;
        double last_time_updated = -1;
        double total_movement = 0.;

    private:
        esekfom::esekf<state_ikfom, 12, input_ikfom> IKFoM_KF;
    
    // Methods

    public:
        Localizator();
        // void reset(int NUM_ITERS=-1);
        void reset(int NUM_ITERS=-1, Eigen::Vector3f initial_pos = Eigen::Vector3f::Zero(), Eigen::Matrix3f initial_R = Eigen::Matrix3f::Identity());

        void localize(const Points&, double time);
        void calculate_H(const state_ikfom&, const Matches&, Eigen::MatrixXd& H, Eigen::VectorXd& h);
        
        bool is_loop_closed(double t);

        void propagate_to(double t);
        State latest_state();

    private:
        void init_IKFoM(int NUM_ITERS=-1, Eigen::Vector3f initial_pos = Eigen::Vector3f::Zero(), Eigen::Matrix3f initial_R = Eigen::Matrix3f::Identity());
        void init_IKFoM_state(const Eigen::Vector3f&, const Eigen::Matrix3f&);
        void IKFoM_update(const Points&);
        
        void propagate(const IMU& imu);
        const state_ikfom& get_x() const;

        bool enough_trajectory(double t);
        bool around_origin(double t);


    // Singleton pattern

    public:
        static Localizator& getInstance();
    
    private:
        // Delete copy/move so extra instances can't be created/moved.
        Localizator(const Localizator&) = delete;
        Localizator& operator=(const Localizator&) = delete;
        Localizator(Localizator&&) = delete;
        Localizator& operator=(Localizator&&) = delete;

};