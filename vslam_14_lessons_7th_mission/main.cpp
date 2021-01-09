//
// Created by sph on 2021/1/5.
//

#include "../points_cloud_display_with_pcl/cloud_viewer.h"
#include "data.h"
#include "BundleAdjustment.h"

int main(int argc, char **argv) {

    std::string file_name = "../problem-49-7776-pre.txt";

    // Load data from dataset files
//    DATA data(file_name);

    // step 1: 构造实例，完成数据读取
    BUNDLEADJUSTMENT myBA(file_name);

    // step 2: 初始化 g2o

    /* step 2.1:   定义线性求解器 LinearSolver */
    // step A: 定义线性求解器的矩阵类型，为什么是9,3?装什么东西?
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3> > BalBlockSolver;
    // step B: 利用上面的矩阵类型来定义线性求解器
    std::unique_ptr<g2o::LinearSolver<BalBlockSolver::PoseMatrixType>> linearSolver;

    /* step 2.2:   定义优化器 SparseOptimizer */
    // step A:     配置优化算法就是 SparseOptimizer.setAlgorithm(solver) 里面的 solver
    // 配置 solver 用的线性求解器里面的数据类型是 CHOLMOD 还是 Eigen
#ifdef G2O_HAVE_CHOLMOD
    string choleskySolverName = "CHOLMOD";
    typedef g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType> BalLinearSolver;
#else
    string choleskySolverName = "Eigen";
  typedef g2o::LinearSolverEigen<BalBlockSolver::PoseMatrixType> BalLinearSolver;
#endif
    typedef g2o::LinearSolverPCG<BalBlockSolver::PoseMatrixType> BalLinearSolverPCG;
    // 配置 solver 用的线性求解器，是 PCG 还是 Cholesky
    bool usePCG = false;
    if (usePCG) {
        cout << "Using PCG" << endl;
        linearSolver = g2o::make_unique<BalLinearSolverPCG>();
    } else {
        cout << "Using Cholesky: " << choleskySolverName << endl;
        auto cholesky = g2o::make_unique<BalLinearSolver>();
        cholesky->setBlockOrdering(true);
        linearSolver = std::move(cholesky);
    }
    // 配置 solver 选择 Levenberg 方法作为优化方法
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BalBlockSolver>(std::move(linearSolver)));

    // step B: 配置优化器算法是上面定义好的 solver
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // step C: 配置优化器其他选项
    std::string statsFilename = "../result.txt";
    if (statsFilename.size() > 0) {
        optimizer.setComputeBatchStatistics(true);
    }

    int id = 0;

    // step 3: 添加相机节点
    vector<VertexCameraBAL*> cameras;
    // vector 的reserve增加了vector的capacity，但是它的size没有改变！而resize改变了vector的capacity同时也增加了它的size！
    // 原因如下：
    // reserve是容器预留空间，但在空间内不真正创建元素对象，所以在没有添加新的对象之前，不能引用容器内的元素。加入新的元素时，
    // 要调用push_back()/insert()函数。
    // resize是改变容器的大小，且在创建对象，因此，调用这个函数之后，就可以引用容器内的对象了，因此当加入新的元素时，用operator[]操作符，
    // 或者用迭代器来引用元素对象。此时再调用push_back()函数，是加在这个新的空间后面的。
    cameras.reserve(myBA.data_.NumCameras());
    for (int i = 0; i < myBA.data_.NumCameras(); ++i, ++id) {
        VertexCameraBAL* cam = new VertexCameraBAL;
        cam->setId(id);
        optimizer.addVertex(cam);
        cameras.push_back(cam);
    }

    // step 4: 添加世界坐标节点
    vector<VertexPointBAL*> points;
    points.reserve(myBA.data_.NumPoints());
    for (int i = 0; i < myBA.data_.NumPoints(); ++i, ++id) {
        VertexPointBAL* p = new VertexPointBAL;
        p->setId(id);
        p->setMarginalized(true);
        bool addedVertex = optimizer.addVertex(p);
        if (! addedVertex) {
            cerr << "failing adding vertex" << endl;
            return -1;
        }
        points.push_back(p);
    }

    // step 5: 读取观测数值
    for (int i = 0; i < myBA.data_.NumObservations(); ++i) {
        int camIndex = myBA.data_.GetObservation(i).camera_index;
        int pointIndex = myBA.data_.GetObservation(i).point_index;
        double obsX = myBA.data_.GetObservation(i).xy(0);
        double obsY = myBA.data_.GetObservation(i).xy(1);

        assert(camIndex >= 0 && (size_t)camIndex < myBA.data_.NumCameras() && "Index out of bounds");
        VertexCameraBAL* cam = cameras[camIndex];
        assert(pointIndex >= 0 && (size_t)pointIndex < myBA.data_.NumPoints() && "Index out of bounds");
        VertexPointBAL* point = points[pointIndex];

        EdgeObservationBAL* e = new EdgeObservationBAL;
        e->setVertex(0, cam);
        e->setVertex(1, point);
        e->setInformation(Eigen::Matrix2d::Identity());
        e->setMeasurement(Eigen::Vector2d(obsX, obsY));
        bool addedEdge = optimizer.addEdge(e);
        if (! addedEdge) {
            cerr << "error adding edge" << endl;
            return -1;
        }
    }

    // step 6: 配置相机参数
    Eigen::VectorXd cameraParameter(9);
    for (int i = 0; i < myBA.data_.NumCameras(); ++i) {
        VertexCameraBAL* cam = cameras[i];
        cameraParameter = myBA.data_.cameras_[i];
        cam->setEstimate(cameraParameter);
    }

    // step 7: 读取观测点坐标
    Eigen::Vector3d p;
    for (int i = 0; i < myBA.data_.NumPoints(); ++i) {
        VertexPointBAL* point = points[i];
        p = myBA.data_.points_[i];
        point->setEstimate(p);
    }

    // step 8: 开始图优化
    cout << "Initializing ... " << flush;
    optimizer.initializeOptimization();
    cout << "done." << endl;
    optimizer.setVerbose(true);
    cout << "Start to optimize" << endl;
    optimizer.optimize(50);

    // step 9: 输出结果到文件中
    if (statsFilename!=""){
        cerr << "writing stats to file \"" << statsFilename << "\" ... ";
        ofstream fout(statsFilename.c_str());
        const BatchStatisticsContainer& bsc = optimizer.batchStatistics();
        for (size_t i=0; i<bsc.size(); i++)
            fout << bsc[i] << endl;
        cerr << "done." << endl;
    }

    // dump the points
    std::string outputFilename = "../points_view.wrl";
    if (outputFilename.size() > 0) {
        ofstream fout(outputFilename.c_str()); // loadable with meshlab
        fout
                << "#VRML V2.0 utf8\n"
                << "Shape {\n"
                << "  appearance Appearance {\n"
                << "    material Material {\n"
                << "      diffuseColor " << 1 << " " << 0 << " " << 0 << "\n"
                << "      ambientIntensity 0.2\n"
                << "      emissiveColor 0.0 0.0 0.0\n"
                << "      specularColor 0.0 0.0 0.0\n"
                << "      shininess 0.2\n"
                << "      transparency 0.0\n"
                << "    }\n"
                << "  }\n"
                << "  geometry PointSet {\n"
                << "    coord Coordinate {\n"
                << "      point [\n";
        for (vector<VertexPointBAL*>::const_iterator it = points.begin(); it != points.end(); ++it) {
            fout << (*it)->estimate().transpose() << endl;
        }
        fout << "    ]\n" << "  }\n" << "}\n" << "  }\n";
    }

    return 0;
}