//
// Created by sph on 2021/1/5.
//

#ifndef INC_7_2_DATA_H
#define INC_7_2_DATA_H

#include <fstream>
#include <iostream>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <vector>

typedef Eigen::Matrix<double, 9, 1> Vector9d;

// 一次观测，即图像中的一个特征点坐标和观测到它的相机以及对应的3D空间点
struct Observation {
    int camera_index;
    int point_index;
    Eigen::Vector2d xy;
};

class DATA {
public:
    DATA();

    DATA(std::string file_name);

    ~DATA();

    bool LoadData();

    inline int NumCameras() const
    {
        return num_cameras_;
    }

    inline int NumPoints() const
    {
        return num_points_;
    }

    inline int NumObservations() const
    {
        return num_observations_;
    }

    inline double* GetCamera(int index)
    {
        return cameras_[index].data();
    }

    inline double* GetPoint(int index)
    {
        return points_[index].data();
    }

    inline Observation GetObservation(int index) const
    {
        return observations_[index];
    }

    std::vector<Vector9d> cameras_;
    std::vector<Eigen::Vector3d> points_;

private:
    std::string file_name_;

    int num_cameras_;
    int num_points_;
    int num_observations_;

    std::vector<Observation> observations_;
};


#endif //INC_7_2_DATA_H
