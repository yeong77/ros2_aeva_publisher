#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include "doppler_icp_node.h"

using namespace open3d;
using namespace open3d::pipelines::registration;

open3d::pipelines::registration::RegistrationResult DopplerICPRealtime::DopplerICP(
    const std::shared_ptr<geometry::PointCloud>& source,
    const std::shared_ptr<geometry::PointCloud>& target,
    const std::unordered_map<std::string, double>& params,
    const Eigen::Matrix4d& init_transform = Eigen::Matrix4d::Identity())
{
    // 1. Downsample
    auto source_in_S_down = source->UniformDownSample(static_cast<size_t>(params.at("downsample_factor")));
    auto target_in_S_down = target->UniformDownSample(static_cast<size_t>(params.at("downsample_factor")));

    // 2. Transform to vehicle frame
    auto source_in_V = std::make_shared<geometry::PointCloud>(*source_in_S_down);
    auto target_in_V = std::make_shared<geometry::PointCloud>(*target_in_S_down);

    Eigen::Matrix4d T_V_to_S = Eigen::Matrix4d::Identity();
    source_in_V->Transform(T_V_to_S);
    target_in_V->Transform(T_V_to_S);

    // 3. Compute direction vectors (unit vectors)
    std::vector<Eigen::Vector3d> source_directions_in_S;
    for (const auto& pt : source_in_S_down->points_) {
        source_directions_in_S.push_back(pt.normalized());
    }

    // 4. Compute normals
    target_in_V->EstimateNormals(
        geometry::KDTreeSearchParamHybrid(params.at("normals_radius"),
                                          static_cast<int>(params.at("normals_max_nn"))));

    // 5. Create robust loss kernels
    auto geometric_kernel = std::make_shared<TukeyLoss>(params.at("geometric_k"));
    auto doppler_kernel = std::make_shared<TukeyLoss>(params.at("doppler_k"));

    // 6. Create estimation object
    TransformationEstimationForDopplerICP estimation(
        params.at("lambda_doppler"),
        static_cast<bool>(params.at("reject_outliers")),
        params.at("outlier_thresh"),
        static_cast<int>(params.at("rejection_min_iters")),
        static_cast<int>(params.at("geometric_min_iters")),
        static_cast<int>(params.at("doppler_min_iters")),
        geometric_kernel,
        doppler_kernel
    );

    // 7. Convergence criteria
    ICPConvergenceCriteria criteria;
    criteria.relative_fitness_ = params.at("convergence_thresh");
    criteria.relative_rmse_ = params.at("convergence_thresh");
    criteria.max_iteration_ = static_cast<int>(params.at("max_iters"));

    // 8. Run Doppler ICP
    auto result = open3d::pipelines::registration::RegistrationDopplerICP(
        *source_in_V,
        *target_in_V,
        source_directions_in_S,
        params.at("max_corr_distance"),
        init_transform,
        estimation,
        criteria,
        params.at("period"),
        T_V_to_S
    );

    return result;
}
