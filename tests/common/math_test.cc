#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <initializer_list>
#include <type_traits>

#include "livo_atlas/common/math_types.h"
#include "livo_atlas/common/so3_math.h"

namespace livo_atlas {
namespace {

static_assert(std::is_same<Vector2f, Eigen::Vector2f>::value);
static_assert(std::is_same<Vector2d, Eigen::Vector2d>::value);
static_assert(std::is_same<Vector3d, Eigen::Vector3d>::value);
static_assert(std::is_same<Matrix3d, Matrix3d>::value);
static_assert(std::is_same<Eigen::Vector3f, Eigen::Vector3f>::value);
static_assert(std::is_same<Matrix3f, Matrix3f>::value);

template <typename T>
class So3MathTest : public testing::Test {
 public:
  using Vector = Eigen::Matrix<T, 3, 1>;
  using Matrix = Eigen::Matrix<T, 3, 3>;
  static constexpr double kTolerance =
      std::is_same<T, double>::value ? 1e-12 : 1e-6;
};

using ScalarTypes = testing::Types<double, float>;
TYPED_TEST_SUITE(So3MathTest, ScalarTypes);

TYPED_TEST(So3MathTest, PreservesSmallAngleBranches) {
  using T = TypeParam;
  using Vector = typename TestFixture::Vector;
  for (const double magnitude : {0.5e-7, 2e-7}) {
    const Vector angle(0, 0, static_cast<T>(magnitude));
    EXPECT_EQ(Exp(Vector(angle)).isIdentity(0), magnitude < 1e-7);
    EXPECT_EQ(Exp(angle, T(1)).isIdentity(0), magnitude < 1e-7);
  }
  for (const double magnitude : {0.5e-5, 2e-5}) {
    EXPECT_EQ(Exp(T(0), T(0), static_cast<T>(magnitude)).isIdentity(0),
              magnitude < 1e-5);
  }
  const Vector angle(0, 0, T(1e-6));
  EXPECT_TRUE(Exp(T(0), T(0), T(1e-6)).isIdentity(0));
  EXPECT_FALSE(Exp(Vector(angle)).isIdentity(0));
  EXPECT_FALSE(Exp(Vector::UnitZ().eval(), 1e-8).isIdentity(0));
  EXPECT_TRUE(Exp(Vector(0, 0, T(0.5e-7)), T(1e9)).isIdentity(0));
}

TYPED_TEST(So3MathTest, RotatesAroundKnownAxis) {
  using T = TypeParam;
  using Vector = typename TestFixture::Vector;
  using Matrix = typename TestFixture::Matrix;
  const T angle = T(0.25);
  Matrix expected;
  expected << std::cos(angle), -std::sin(angle), 0, std::sin(angle),
      std::cos(angle), 0, 0, 0, 1;
  EXPECT_TRUE(
      Exp(Vector(0, 0, angle)).isApprox(expected, TestFixture::kTolerance));
  EXPECT_TRUE(Exp(Vector::UnitZ().eval(), angle)
                  .isApprox(expected, TestFixture::kTolerance));
  EXPECT_TRUE(
      Exp(T(0), T(0), angle).isApprox(expected, TestFixture::kTolerance));
}

TYPED_TEST(So3MathTest, PreservesLogBranches) {
  using T = TypeParam;
  using Vector = typename TestFixture::Vector;
  using Matrix = typename TestFixture::Matrix;
  EXPECT_TRUE(Log(Matrix(Matrix::Identity())).isZero(0));
  for (const double angle : {0.0005, 0.25}) {
    const Matrix rotation =
        Eigen::AngleAxis<T>(T(angle), Vector::UnitZ()).toRotationMatrix();
    const Vector result = Log(rotation);
    EXPECT_NEAR(result.x(), 0, TestFixture::kTolerance);
    EXPECT_NEAR(result.y(), 0, TestFixture::kTolerance);
    EXPECT_NEAR(result.z(), angle < 0.001 ? std::sin(angle) : angle,
                TestFixture::kTolerance);
  }
}

TYPED_TEST(So3MathTest, PreservesEulerSingularityConvention) {
  using T = TypeParam;
  using Vector = typename TestFixture::Vector;
  using Matrix = typename TestFixture::Matrix;
  const Matrix yaw =
      Eigen::AngleAxis<T>(T(0.25), Vector::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(RotationMatrixToEuler(yaw).isApprox(Vector(0, 0, T(0.25)),
                                                  TestFixture::kTolerance));
  const T half_pi = T(std::acos(-1.0) / 2);
  const Matrix pitch =
      Eigen::AngleAxis<T>(half_pi, Vector::UnitY()).toRotationMatrix();
  const Vector result = RotationMatrixToEuler(pitch);
  EXPECT_NEAR(result.x(), 0, TestFixture::kTolerance);
  EXPECT_NEAR(result.y(), half_pi, TestFixture::kTolerance);
  EXPECT_EQ(result.z(), 0);
}

}  // namespace
}  // namespace livo_atlas
