#include "foot_force_estimator.h"

#include <algorithm>
#include <cmath>

namespace {

bool finiteMatrix(const float matrix[3][3]) {
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            if (!std::isfinite(matrix[row][column])) return false;
    return true;
}

bool finiteVector(const float vector[3]) {
    return std::isfinite(vector[0])
        && std::isfinite(vector[1])
        && std::isfinite(vector[2]);
}

float determinant3(const float matrix[3][3]) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2]
                         - matrix[1][2] * matrix[2][1])
         - matrix[0][1] * (matrix[1][0] * matrix[2][2]
                         - matrix[1][2] * matrix[2][0])
         + matrix[0][2] * (matrix[1][0] * matrix[2][1]
                         - matrix[1][1] * matrix[2][0]);
}

// 对称正定3x3矩阵的Cholesky求解；DLS对角阻尼保证非奇异。
bool solvePositiveDefinite3(const float matrix[3][3],
                            const float rhs[3], float solution[3]) {
    float lower[3][3] = {{0.0f}};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column <= row; ++column) {
            float value = matrix[row][column];
            for (int k = 0; k < column; ++k)
                value -= lower[row][k] * lower[column][k];
            if (row == column) {
                if (!(value > 1.0e-12f) || !std::isfinite(value)) return false;
                lower[row][column] = std::sqrt(value);
            } else {
                lower[row][column] = value / lower[column][column];
            }
        }
    }

    float intermediate[3] = {0.0f};
    for (int row = 0; row < 3; ++row) {
        float value = rhs[row];
        for (int k = 0; k < row; ++k) value -= lower[row][k] * intermediate[k];
        intermediate[row] = value / lower[row][row];
    }
    for (int row = 2; row >= 0; --row) {
        float value = intermediate[row];
        for (int k = row + 1; k < 3; ++k) value -= lower[k][row] * solution[k];
        solution[row] = value / lower[row][row];
    }
    return finiteVector(solution);
}

}  // namespace

FootForceEstimator::FootForceEstimator(const FootForceEstimatorConfig& config)
    : config_(config) {}

FootForceEstimate FootForceEstimator::estimate(
    const float jacobian[3][3], const float joint_torque[3]) const {
    FootForceEstimate output;
    if (!jacobian || !joint_torque || !finiteMatrix(jacobian)
        || !finiteVector(joint_torque) || !std::isfinite(config_.damping)
        || config_.damping <= 0.0f) {
        return output;
    }

    float column_norm_product = 1.0f;
    for (int column = 0; column < 3; ++column) {
        float norm_sq = 0.0f;
        for (int row = 0; row < 3; ++row)
            norm_sq += jacobian[row][column] * jacobian[row][column];
        column_norm_product *= std::sqrt(norm_sq);
    }
    output.normalized_determinant = column_norm_product > 1.0e-12f
        ? std::min(1.0f, std::abs(determinant3(jacobian))
                         / column_norm_product)
        : 0.0f;

    float normal_matrix[3][3] = {{0.0f}};
    float rhs[3] = {0.0f};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int joint = 0; joint < 3; ++joint)
                normal_matrix[row][column] +=
                    jacobian[row][joint] * jacobian[column][joint];
        }
        // 静力平衡：tau_motor + J^T * F_ground ~= 0。
        for (int joint = 0; joint < 3; ++joint)
            rhs[row] -= jacobian[row][joint] * joint_torque[joint];
    }
    const float damping_sq = config_.damping * config_.damping;
    for (int axis = 0; axis < 3; ++axis) normal_matrix[axis][axis] += damping_sq;

    if (!solvePositiveDefinite3(normal_matrix, rhs, output.force_body))
        return output;

    float force_norm_sq = 0.0f;
    for (float value : output.force_body) force_norm_sq += value * value;
    output.force_norm = std::sqrt(force_norm_sq);
    output.normal_force = std::max(0.0f, output.force_body[2]);

    float residual_sq = 0.0f;
    for (int joint = 0; joint < 3; ++joint) {
        float residual = joint_torque[joint];
        for (int axis = 0; axis < 3; ++axis)
            residual += jacobian[axis][joint] * output.force_body[axis];
        residual_sq += residual * residual;
    }
    output.torque_residual = std::sqrt(residual_sq);
    output.valid = finiteVector(output.force_body)
        && std::isfinite(output.force_norm)
        && std::isfinite(output.torque_residual)
        && output.normalized_determinant >= config_.min_normalized_determinant
        && output.force_norm <= config_.max_force_norm
        && output.torque_residual <= config_.max_torque_residual;
    return output;
}
