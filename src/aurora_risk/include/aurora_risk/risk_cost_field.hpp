#pragma once

#include "aurora_prediction/kinematic_predictor.hpp"
#include "aurora_risk/risk_context.hpp"
#include "aurora_trajectory/static_bspline_optimizer.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace aurora::risk {

struct DynamicRiskCostFieldOptions {
  double vehicle_radius{0.65};
  double sigma_multiplier{3.0};
  double warning_clearance{0.5};
  std::size_t max_obstacles{1000};
};

// Evaluates the soft part of the dynamic risk policy. It never replaces
// DynamicRiskEvaluator: that evaluator remains the publish-time hard gate.
class DynamicRiskCostField {
public:
  DynamicRiskCostField(std::vector<aurora::prediction::PredictionResult> predictions,
                       RiskContext context,
                       DynamicRiskCostFieldOptions options = {});

  const DynamicRiskCostFieldOptions &options() const noexcept { return options_; }

  aurora::trajectory::RiskCostEvaluation evaluate(
      double absolute_stamp, const Eigen::Vector3d &position) const;

private:
  static void validateOptions(const DynamicRiskCostFieldOptions &options);

  std::vector<aurora::prediction::PredictionResult> predictions_;
  RiskContext context_;
  DynamicRiskCostFieldOptions options_;
};

}  // namespace aurora::risk
