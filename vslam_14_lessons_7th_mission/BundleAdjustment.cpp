//
// Created by sph on 2021/1/5.
//

#include <opencv2/core/types.hpp>
#include "BundleAdjustment.h"

BUNDLEADJUSTMENT::BUNDLEADJUSTMENT() {
    std::cout << "Construct BUNDLEADJUSTMENT without loading data." << std::endl;
}

BUNDLEADJUSTMENT::BUNDLEADJUSTMENT(std::string file_name) : data_(file_name) {}

//void BUNDLEADJUSTMENT::bundleAdjustment(
//        const std::vector <cv::Point3f> points_3d,
//        const std::vector <cv::Point2f> points_2d,
//        cv::Mat &K) {
//    // creat g2o
//    // new g2o version. Ref:https://www.cnblogs.com/xueyuanaichiyu/p/7921382.html
//
//    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> > Block;  // pose 维度为 6, landmark 维度为 3
//    // 第1步：创建一个线性求解器LinearSolver
//    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
//
//    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
//    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
//
//    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
//    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
//            std::unique_ptr<Block>(solver_ptr));
//
//    // 第4步：创建稀疏优化器
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm(solver);
//
////    // old g2o version
////    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
////    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
////    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
////    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
////    g2o::SparseOptimizer optimizer;
////    optimizer.setAlgorithm ( solver );
//
//    // 第5步：定义图的顶点和边。并添加到SparseOptimizer中
//
//    // ----------------------开始你的代码：设置并添加顶点，初始位姿为单位矩阵
//
//    // ----------------------结束你的代码
//
//
//    // 设置相机内参
//    g2o::CameraParameters *camera = new g2o::CameraParameters(
//            K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
//    camera->setId(0);
//    optimizer.addParameter(camera);
//
//    // 设置边
//    int index = 1;
//    for (const cv::Point2f p:points_2d) {
//        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
//        edge->setId(index);
//        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *> ( optimizer.vertex(index)));
//        edge->setVertex(1, pose);
//        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));  //设置观测值
//        edge->setParameterId(0, 0);
//        edge->setInformation(Eigen::Matrix2d::Identity());
//        optimizer.addEdge(edge);
//        index++;
//    }
//
//
//    // 第6步：设置优化参数，开始执行优化
//    optimizer.setVerbose(false);
//    optimizer.initializeOptimization();
//    optimizer.optimize(100);
//
//    // 输出优化结果
//    std::cout << std::endl << "after optimization:" << std::endl;
//    std::cout << "T=" << std::endl << Eigen::Isometry3d(pose->estimate()).matrix() << std::endl;
//}
