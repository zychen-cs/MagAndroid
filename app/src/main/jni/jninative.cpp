//#include <jni.h>
//#include <string>
//
//#include "ceres.h"
//#include <glog/logging.h>
//
//using ceres::AutoDiffCostFunction;
//using ceres::CostFunction;
//using ceres::Problem;
//using ceres::Solver;
//using ceres::Solve;
//
//// A templated cost functor that implements the residual r = 10 -
//// x. The method operator() is templated so that we can then use an
//// automatic differentiation wrapper around it to generate its
//// derivatives.
//struct CostFunctor {
//    template <typename T> bool operator()(const T* const x, T* residual) const {
//        residual[0] = 10.0 - x[0];
//        return true;
//    }
//};
//
//extern "C" JNIEXPORT jstring JNICALL
//Java_com_example_magandorid_mainActivity_stringFromJNI(
//        JNIEnv* env,
//        jobject /* this */) {
//    std::string hello = "Hello Ceres";
//
//    // The variable to solve for with its initial value. It will be
//    // mutated in place by the solver.
//    double x = 0.5;
//    const double initial_x = x;
//
//    // Build the problem.
//    Problem problem;
//
//    // Set up the only cost function (also known as residual). This uses
//    // auto-differentiation to obtain the derivative (jacobian).
//    CostFunction* cost_function =
//            new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
//    problem.AddResidualBlock(cost_function, NULL, &x);
//
//    // Run the solver!
//    Solver::Options options;
//    options.minimizer_progress_to_stdout = true;
//    Solver::Summary summary;
//    Solve(options, &problem, &summary);
//
//    std::cout << summary.BriefReport() << "\n";
//    std::cout << "x : " << initial_x
//              << " -> " << x << "\n";
//
//    return env->NewStringUTF(summary.BriefReport().c_str());
//}

#include <jni.h>
#include <string>
//#include "ceres.h"
#include "ceres.h"
//#include <gflags/gflags.h>
#include <glog/logging.h>
#include <math.h>
//#include <pybind11/numpy.h>
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <android/log.h>

#define  LOG_TAG    "your-log-tag"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
const int kNumObservations = 8;

#define STRINGIFY(x) #x
#define PI 3.1415926
#define MACRO_STRINGIFY(x) STRINGIFY(x)
//namespace py = pybind11;

void printEigenVector(const Eigen::VectorXd& vec) {
    for(int i = 0; i < vec.size(); ++i) {
//        __android_log_print(ANDROID_LOG_INFO, "EigenVector", "[%d]: %f", i, vec[i]);
        LOGE("转换后的数据", vec.data());
        LOGE("转换后的大小%d", vec.size());

    }
}

struct Cost_FixedM_1mag {
    Cost_FixedM_1mag(double x, double y, double z, double x2, double y2,
                     double z2, double m)
            : Bx(x),
              By(y),
              Bz(z),
              Xs(x2),
              Ys(y2),
              Zs(z2),
              M{m} {}  // init the sensor position and the magnitude reading.
    template <typename T>
    bool operator()(const T *const x, const T *const y, const T *const z,
                    const T *const theta, const T *const phy, const T *const Gx,
                    const T *const Gy, const T *const Gz, T *residual)
    const {  // x y z is the coordinates of magnate j, m is the attributes of
        // magate j, theta phy is the orientation of the magnate
        Eigen::Matrix<T, 3, 1> VecM =
                Eigen::Matrix<T, 3, 1>(sin(theta[0]) * cos(phy[0]),
                                       sin(theta[0]) * sin(phy[0]), cos(theta[0])) *
                1e-7 * exp(M);
        Eigen::Matrix<T, 3, 1> VecR =
                Eigen::Matrix<T, 3, 1>(Xs - x[0], Ys - y[0], Zs - z[0]);
        T NormR = VecR.norm();
        Eigen::Matrix<T, 3, 1> B =
                (3.0 * VecR * (VecM.transpose() * VecR) / pow(NormR, 5) -
                 VecM /
                 pow(NormR, 3));  // convert it's unit to correspond with the input
        // std::cout << "B= " << (B(0, 0) + Gx[0]) * 1e6 << "\t" << (B(1, 0) +
        // Gy[0]) * 1e6 << "\t" << (B(2, 0) + Gz[0]) * 1e6 << "\n"; std::cout <<
        // B(0) << '\n'
        //           << B(1) << '\n'
        //           << B(2) << std::endl;
        residual[0] = (B(0, 0) + Gx[0]) * 1e6 - Bx;
        residual[1] = (B(1, 0) + Gy[0]) * 1e6 - By;
        residual[2] = (B(2, 0) + Gz[0]) * 1e6 - Bz;
        // std::cout << residual[0] << '\t' << residual[1] << '\t' << residual[2] <<
        // std::endl;
        return true;
    }

private:
    const double Bx;
    const double By;
    const double Bz;
    const double Xs;
    const double Ys;
    const double Zs;
    const double M;
};


extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_magandorid_FindDeviceMainActivity_00024Companion_solve_11mag(JNIEnv *env,
                                                                              jobject thiz,
                                                                              jdoubleArray readings1,
                                                                              jdoubleArray psensor1,
                                                                              jdoubleArray init_param1) {

    jsize size = env->GetArrayLength( readings1 );
    std::vector<double> readings(size);
    env->GetDoubleArrayRegion(readings1, jsize{0}, size, &readings[0] );
    for (int i = 0; i <size; ++i) {
        LOGE("转换后的数据 readings %f", readings.at(i));
    }
    jsize size1 = env->GetArrayLength( psensor1 );
    std::vector<double> psensor(size1);
    env->GetDoubleArrayRegion(psensor1, jsize{0}, size1, &psensor[0] );

    jsize size2 = env->GetArrayLength( init_param1 );
    std::vector<double> init_param(size2);
    env->GetDoubleArrayRegion(init_param1, jsize{0}, size2,&init_param[0] );
    //     std::vector<float> test_vector = { 2,1,3 };
    // Eigen::MatrixXf readings_vec = Eigen::Map<Eigen::Matrix<double, 8, 3>
    // >(readings.data()); Eigen::MatrixXf pSensor_vec =
    // Eigen::Map<Eigen::Matrix<double, 8, 3> >(pSensor.data());
//    jint len1 = env->GetArrayLength(static_cast<jarray>(readings));
//    jint len2 = env->GetArrayLength(static_cast<jarray>(psensor));
    LOGD("size %d", readings.size());
    Eigen::VectorXd readings_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            readings.data(), readings.size());

    Eigen::VectorXd pSensor_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            psensor.data(), psensor.size());
    printEigenVector(readings_vec);
    // Eigen::MatrixXd readings_vec_1(&readings[0], 8, 3);
    // Eigen::MatrixXd pSensor_vec_1(&pSensor[0], 8, 3);
    // Eigen::Map<Eigen::MatrixXd> readings_vec(readings_vec_1.data(), 3, 8);
    // Eigen::Map<Eigen::MatrixXd> pSensor_vec(pSensor_vec_1.data(), 3, 8);
    // readings_vec = readings_vec.transpose();
    // pSensor_vec = pSensor_vec.transpose();
    // std::cout
    //     << "readings_vec: " << readings_vec << "\n";
    // std::cout << "pSensor_vec: " << pSensor_vec << "\n";

    double Gx = init_param[0];
    double Gy = init_param[1];
    double Gz = init_param[2];
    double m = init_param[3];
    double x = init_param[4];
    double y = init_param[5];
    double z = init_param[6];
    double theta = init_param[7];
    double phy = init_param[8];
    LOGD("phy %d", phy);
    // std::cout << "Initial x: " << x << " y: " << y << " z: " << z << " m: " <<
    // m << " theta: " << theta << " phy: " << phy << " Gx: " << Gx << " Gy: " <<
    // Gy << " Gz: " << Gz << "\n";
    // Problem problem;
    // for (int i = 0; i < int(pSensor_vec.size() / 3); ++i) {
    // problem.AddResidualBlock(
    //     new AutoDiffCostFunction<Cost, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
    //         new Cost(testdata(i, 0), testdata(i, 1), testdata(i, 2),
    //         sPosition(i, 0), sPosition(i, 1), sPosition(i, 2))),
    //     NULL, &x, &y, &z, &m, &theta, &phy, &Gx, &Gy, &Gz);

//求m值不固定代码
    Problem problem;
    for (int i = 0; i < int(pSensor_vec.size() / 3); ++i) {
//        problem.AddResidualBlock(
//                new AutoDiffCostFunction<Cost_1mag, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
//                        new Cost_1mag(readings_vec[i * 3], readings_vec[i * 3 + 1],
//                                      readings_vec[i * 3 + 2], pSensor_vec[i * 3],
//                                      pSensor_vec[i * 3 + 1], pSensor_vec[i * 3 + 2])),
//                NULL, &x, &y, &z, &m, &theta, &phy, &Gx, &Gy, &Gz);
//    }

        //求m值固定代码
        problem.AddResidualBlock(
                new AutoDiffCostFunction<Cost_FixedM_1mag, 3, 1, 1, 1, 1, 1, 1, 1, 1>(
                        new Cost_FixedM_1mag(readings_vec[i * 3], readings_vec[i * 3 + 1],
                                             readings_vec[i * 3 + 2], pSensor_vec[i * 3],
                                             pSensor_vec[i * 3 + 1], pSensor_vec[i * 3 + 2],
                                             m)),

                NULL, &x, &y, &z, &theta, &phy, &Gx, &Gy, &Gz);
    }

    problem.SetParameterLowerBound(&theta, 0, 0);
    problem.SetParameterUpperBound(&theta, 0, PI);

    problem.SetParameterLowerBound(&phy, 0, 0);
    problem.SetParameterUpperBound(&phy, 0, 2*PI);

//     problem.SetParameterLowerBound(&x, 0, -10);
//     problem.SetParameterUpperBound(&x, 0, 10);
//
//     problem.SetParameterLowerBound(&y, 0, -10);
//     problem.SetParameterUpperBound(&y, 0, 10);
//
//     problem.SetParameterLowerBound(&z, 0, -10);
//     problem.SetParameterUpperBound(&z, 0, 10);


    Solver::Options options;
    options.max_num_iterations = 1e7;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = std::thread::hardware_concurrency();
    // options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
//    options.max_num_iterations = 1e5;
    // options.min_relative_decrease = 1e-16;
    // options.max_num_consecutive_invalid_steps = 1e6;
    // options.function_tolerance = 1e-32;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";
    // std::cout << "Initial x: " << 0.0 << " y: " << 0.0 << " z: " << 0.0 << " m:
    // " << 0.0 << " theta: " << 0.0 << " phy: " << 0.0 << "\n"; std::cout <<
    // "Final x: " << x << " y: " << y << " z: " << z << " m: " << m << " theta: "
    // << theta << " phy: " << phy << " Gx: " << Gx << " Gy: " << Gy << " Gz: " <<
    // Gz << "\n";

    // set params
    std::vector<double> result_vec = {Gx, Gy, Gz, m, x, y, z, theta, phy};
    LOGD("x,y,z %f %f %f", x, y, z);
    for (auto& value : result_vec) {
        LOGD("Result value: %f", value);
    }
    LOGD("Solver summary: %s", summary.BriefReport().c_str());


//    std::vector<double> readings(size);
//    env->GetDoubleArrayRegion(readings1, jsize{0}, size, &readings[0] );
    jdoubleArray output = env->NewDoubleArray( result_vec.size() );
    env->SetDoubleArrayRegion( output, 0, result_vec.size(), &result_vec[0] );

    return output;

}
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_magandorid_mainActivity_solve_11mag(JNIEnv *env, jobject thiz,
                                                     jdoubleArray readings1, jdoubleArray psensor1,
                                                     jdoubleArray init_param1) {
    // TODO: implement solve_1mag()
    jsize size = env->GetArrayLength( readings1 );
    std::vector<double> readings(size);
    env->GetDoubleArrayRegion(readings1, jsize{0}, size, &readings[0] );
    for (int i = 0; i <size; ++i) {
        LOGE("转换后的数据 readings %f", readings.at(i));
    }
    jsize size1 = env->GetArrayLength( psensor1 );
    std::vector<double> psensor(size1);
    env->GetDoubleArrayRegion(psensor1, jsize{0}, size1, &psensor[0] );

    jsize size2 = env->GetArrayLength( init_param1 );
    std::vector<double> init_param(size2);
    env->GetDoubleArrayRegion(init_param1, jsize{0}, size2,&init_param[0] );
    //     std::vector<float> test_vector = { 2,1,3 };
    // Eigen::MatrixXf readings_vec = Eigen::Map<Eigen::Matrix<double, 8, 3>
    // >(readings.data()); Eigen::MatrixXf pSensor_vec =
    // Eigen::Map<Eigen::Matrix<double, 8, 3> >(pSensor.data());
//    jint len1 = env->GetArrayLength(static_cast<jarray>(readings));
//    jint len2 = env->GetArrayLength(static_cast<jarray>(psensor));
    LOGD("size %d", readings.size());
    Eigen::VectorXd readings_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            readings.data(), readings.size());

    Eigen::VectorXd pSensor_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            psensor.data(), psensor.size());
    printEigenVector(readings_vec);
    // Eigen::MatrixXd readings_vec_1(&readings[0], 8, 3);
    // Eigen::MatrixXd pSensor_vec_1(&pSensor[0], 8, 3);
    // Eigen::Map<Eigen::MatrixXd> readings_vec(readings_vec_1.data(), 3, 8);
    // Eigen::Map<Eigen::MatrixXd> pSensor_vec(pSensor_vec_1.data(), 3, 8);
    // readings_vec = readings_vec.transpose();
    // pSensor_vec = pSensor_vec.transpose();
    // std::cout
    //     << "readings_vec: " << readings_vec << "\n";
    // std::cout << "pSensor_vec: " << pSensor_vec << "\n";

    double Gx = init_param[0];
    double Gy = init_param[1];
    double Gz = init_param[2];
    double m = init_param[3];
    double x = init_param[4];
    double y = init_param[5];
    double z = init_param[6];
    double theta = init_param[7];
    double phy = init_param[8];
    LOGD("phy %d", phy);
    // std::cout << "Initial x: " << x << " y: " << y << " z: " << z << " m: " <<
    // m << " theta: " << theta << " phy: " << phy << " Gx: " << Gx << " Gy: " <<
    // Gy << " Gz: " << Gz << "\n";
    // Problem problem;
    // for (int i = 0; i < int(pSensor_vec.size() / 3); ++i) {
    // problem.AddResidualBlock(
    //     new AutoDiffCostFunction<Cost, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
    //         new Cost(testdata(i, 0), testdata(i, 1), testdata(i, 2),
    //         sPosition(i, 0), sPosition(i, 1), sPosition(i, 2))),
    //     NULL, &x, &y, &z, &m, &theta, &phy, &Gx, &Gy, &Gz);

//求m值不固定代码
    Problem problem;
    for (int i = 0; i < int(pSensor_vec.size() / 3); ++i) {
//        problem.AddResidualBlock(
//                new AutoDiffCostFunction<Cost_1mag, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
//                        new Cost_1mag(readings_vec[i * 3], readings_vec[i * 3 + 1],
//                                      readings_vec[i * 3 + 2], pSensor_vec[i * 3],
//                                      pSensor_vec[i * 3 + 1], pSensor_vec[i * 3 + 2])),
//                NULL, &x, &y, &z, &m, &theta, &phy, &Gx, &Gy, &Gz);
//    }

        //求m值固定代码
        problem.AddResidualBlock(
                new AutoDiffCostFunction<Cost_FixedM_1mag, 3, 1, 1, 1, 1, 1, 1, 1, 1>(
                        new Cost_FixedM_1mag(readings_vec[i * 3], readings_vec[i * 3 + 1],
                                             readings_vec[i * 3 + 2], pSensor_vec[i * 3],
                                             pSensor_vec[i * 3 + 1], pSensor_vec[i * 3 + 2],
                                             m)),

                NULL, &x, &y, &z, &theta, &phy, &Gx, &Gy, &Gz);
    }

    problem.SetParameterLowerBound(&theta, 0, 0);
    problem.SetParameterUpperBound(&theta, 0, PI);

    problem.SetParameterLowerBound(&phy, 0, 0);
    problem.SetParameterUpperBound(&phy, 0, 2*PI);

//     problem.SetParameterLowerBound(&x, 0, -10);
//     problem.SetParameterUpperBound(&x, 0, 10);
//
//     problem.SetParameterLowerBound(&y, 0, -10);
//     problem.SetParameterUpperBound(&y, 0, 10);
//
//     problem.SetParameterLowerBound(&z, 0, -10);
//     problem.SetParameterUpperBound(&z, 0, 10);


    Solver::Options options;
    options.max_num_iterations = 1e7;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = std::thread::hardware_concurrency();
    // options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
//    options.max_num_iterations = 1e5;
    // options.min_relative_decrease = 1e-16;
    // options.max_num_consecutive_invalid_steps = 1e6;
    // options.function_tolerance = 1e-32;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";
    // std::cout << "Initial x: " << 0.0 << " y: " << 0.0 << " z: " << 0.0 << " m:
    // " << 0.0 << " theta: " << 0.0 << " phy: " << 0.0 << "\n"; std::cout <<
    // "Final x: " << x << " y: " << y << " z: " << z << " m: " << m << " theta: "
    // << theta << " phy: " << phy << " Gx: " << Gx << " Gy: " << Gy << " Gz: " <<
    // Gz << "\n";

    // set params
    std::vector<double> result_vec = {Gx, Gy, Gz, m, x, y, z, theta, phy};
    LOGD("x,y,z %f %f %f", x, y, z);
    for (auto& value : result_vec) {
        LOGD("Result value: %f", value);
    }
    LOGD("Solver summary: %s", summary.BriefReport().c_str());


//    std::vector<double> readings(size);
//    env->GetDoubleArrayRegion(readings1, jsize{0}, size, &readings[0] );
    jdoubleArray output = env->NewDoubleArray( result_vec.size() );
    env->SetDoubleArrayRegion( output, 0, result_vec.size(), &result_vec[0] );

    return output;
}
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_magandorid_CalibrationActivity_00024Companion_solve_11mag(JNIEnv *env,
                                                                           jobject thiz,
                                                                           jdoubleArray readings1,
                                                                           jdoubleArray psensor1,
                                                                           jdoubleArray init_param1) {
    // TODO: implement solve_1mag()
    jsize size = env->GetArrayLength( readings1 );
    std::vector<double> readings(size);
    env->GetDoubleArrayRegion(readings1, jsize{0}, size, &readings[0] );
    for (int i = 0; i <size; ++i) {
        LOGE("转换后的数据 readings %f", readings.at(i));
    }
    jsize size1 = env->GetArrayLength( psensor1 );
    std::vector<double> psensor(size1);
    env->GetDoubleArrayRegion(psensor1, jsize{0}, size1, &psensor[0] );

    jsize size2 = env->GetArrayLength( init_param1 );
    std::vector<double> init_param(size2);
    env->GetDoubleArrayRegion(init_param1, jsize{0}, size2,&init_param[0] );
    //     std::vector<float> test_vector = { 2,1,3 };
    // Eigen::MatrixXf readings_vec = Eigen::Map<Eigen::Matrix<double, 8, 3>
    // >(readings.data()); Eigen::MatrixXf pSensor_vec =
    // Eigen::Map<Eigen::Matrix<double, 8, 3> >(pSensor.data());
//    jint len1 = env->GetArrayLength(static_cast<jarray>(readings));
//    jint len2 = env->GetArrayLength(static_cast<jarray>(psensor));
    LOGD("size %d", readings.size());
    Eigen::VectorXd readings_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            readings.data(), readings.size());

    Eigen::VectorXd pSensor_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
            psensor.data(), psensor.size());
    printEigenVector(readings_vec);
    // Eigen::MatrixXd readings_vec_1(&readings[0], 8, 3);
    // Eigen::MatrixXd pSensor_vec_1(&pSensor[0], 8, 3);
    // Eigen::Map<Eigen::MatrixXd> readings_vec(readings_vec_1.data(), 3, 8);
    // Eigen::Map<Eigen::MatrixXd> pSensor_vec(pSensor_vec_1.data(), 3, 8);
    // readings_vec = readings_vec.transpose();
    // pSensor_vec = pSensor_vec.transpose();
    // std::cout
    //     << "readings_vec: " << readings_vec << "\n";
    // std::cout << "pSensor_vec: " << pSensor_vec << "\n";

    double Gx = init_param[0];
    double Gy = init_param[1];
    double Gz = init_param[2];
    double m = init_param[3];
    double x = init_param[4];
    double y = init_param[5];
    double z = init_param[6];
    double theta = init_param[7];
    double phy = init_param[8];
    LOGD("phy %d", phy);
    // std::cout << "Initial x: " << x << " y: " << y << " z: " << z << " m: " <<
    // m << " theta: " << theta << " phy: " << phy << " Gx: " << Gx << " Gy: " <<
    // Gy << " Gz: " << Gz << "\n";
    // Problem problem;
    // for (int i = 0; i < int(pSensor_vec.size() / 3); ++i) {
    // problem.AddResidualBlock(
    //     new AutoDiffCostFunction<Cost, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
    //         new Cost(testdata(i, 0), testdata(i, 1), testdata(i, 2),
    //         sPosition(i, 0), sPosition(i, 1), sPosition(i, 2))),
    //     NULL, &x, &y, &z, &m, &theta, &phy, &Gx, &Gy, &Gz);

//求m值不固定代码
    Problem problem;
    for (int i = 0; i < int(pSensor_vec.size() / 3); ++i) {
//        problem.AddResidualBlock(
//                new AutoDiffCostFunction<Cost_1mag, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
//                        new Cost_1mag(readings_vec[i * 3], readings_vec[i * 3 + 1],
//                                      readings_vec[i * 3 + 2], pSensor_vec[i * 3],
//                                      pSensor_vec[i * 3 + 1], pSensor_vec[i * 3 + 2])),
//                NULL, &x, &y, &z, &m, &theta, &phy, &Gx, &Gy, &Gz);
//    }

        //求m值固定代码
        problem.AddResidualBlock(
                new AutoDiffCostFunction<Cost_FixedM_1mag, 3, 1, 1, 1, 1, 1, 1, 1, 1>(
                        new Cost_FixedM_1mag(readings_vec[i * 3], readings_vec[i * 3 + 1],
                                             readings_vec[i * 3 + 2], pSensor_vec[i * 3],
                                             pSensor_vec[i * 3 + 1], pSensor_vec[i * 3 + 2],
                                             m)),

                NULL, &x, &y, &z, &theta, &phy, &Gx, &Gy, &Gz);
    }

    problem.SetParameterLowerBound(&theta, 0, 0);
    problem.SetParameterUpperBound(&theta, 0, PI);

    problem.SetParameterLowerBound(&phy, 0, 0);
    problem.SetParameterUpperBound(&phy, 0, 2*PI);

//     problem.SetParameterLowerBound(&x, 0, -10);
//     problem.SetParameterUpperBound(&x, 0, 10);
//
//     problem.SetParameterLowerBound(&y, 0, -10);
//     problem.SetParameterUpperBound(&y, 0, 10);
//
//     problem.SetParameterLowerBound(&z, 0, -10);
//     problem.SetParameterUpperBound(&z, 0, 10);


    Solver::Options options;
    options.max_num_iterations = 1e5;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = std::thread::hardware_concurrency();
    // options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
//    options.max_num_iterations = 1e5;
    // options.min_relative_decrease = 1e-16;
    // options.max_num_consecutive_invalid_steps = 1e6;
    // options.function_tolerance = 1e-32;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";
    // std::cout << "Initial x: " << 0.0 << " y: " << 0.0 << " z: " << 0.0 << " m:
    // " << 0.0 << " theta: " << 0.0 << " phy: " << 0.0 << "\n"; std::cout <<
    // "Final x: " << x << " y: " << y << " z: " << z << " m: " << m << " theta: "
    // << theta << " phy: " << phy << " Gx: " << Gx << " Gy: " << Gy << " Gz: " <<
    // Gz << "\n";

    // set params
    std::vector<double> result_vec = {Gx, Gy, Gz, m, x, y, z, theta, phy};
    LOGD("x,y,z %f %f %f", x, y, z);
    for (auto& value : result_vec) {
        LOGD("Result value: %f", value);
    }
    LOGD("Solver summary: %s", summary.BriefReport().c_str());


//    std::vector<double> readings(size);
//    env->GetDoubleArrayRegion(readings1, jsize{0}, size, &readings[0] );
    jdoubleArray output = env->NewDoubleArray( result_vec.size() );
    env->SetDoubleArrayRegion( output, 0, result_vec.size(), &result_vec[0] );

    return output;
}
