//#include <jni.h>
//#include <string>
//
//#include "ceres/ceres.h"
//#include "glog/logging.h"
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
//Java_com_example_magandorid_MainActivity_stringFromJNI(
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
////    std::string hello = "Hello from C++";
//    std::cout << summary.BriefReport() << "\n";
//    std::cout << "x : " << initial_x
//              << " -> " << x << "\n";
////    return env->NewStringUTF(hello.c_str());
//    return env->NewStringUTF(summary.BriefReport().c_str());
//}




#include <jni.h>
#include <string>
#include <jni.h>

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_magandorid_mainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C"
JNIEXPORT jdoubleArray