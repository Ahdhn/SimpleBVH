#include <SimpleBVH/BVH.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("test_tree", "[tests]")
{
    Eigen::MatrixXd pts(4, 3);
    // clang-format off
    pts << 0, 0, 0,
           3, 0, 0,
           0, 3, 0,
           0, 3, 3;
    // clang-format on

    Eigen::MatrixXi tri(4, 3);
    // clang-format off
    tri << 0, 1, 2,
           0, 1, 3,
           0, 2, 3,
           1, 2, 3;
    // clang-format on

    SimpleBVH::BVH bvh;
    bvh.init(pts, tri, 1e-10);

    std::vector<unsigned int> pairs;
    bvh.intersect_box(
        pts.colwise().minCoeff(), pts.colwise().maxCoeff(), pairs);

    CHECK(pairs.size() == tri.rows());
}

TEST_CASE("test_ray_mesh_intersection", "[tests]")
{

    Eigen::MatrixXd pts(4, 3);
    // clang-format off
    pts << 0, 0, 0,
           3, 0, 0,
           0, 3, 0,
           0, 3, 3;
    // clang-format on

    Eigen::MatrixXi tri(4, 3);
    // clang-format off
    tri << 0, 1, 2,
           0, 1, 3,
           0, 2, 3,
           1, 2, 3;
    // clang-format on

    SimpleBVH::BVH bvh;
    bvh.init(pts, tri, 1e-10);

    Eigen::Vector3d origin(1.5, -1, 1.5);
    Eigen::Vector3d dir(0, 1, 0);

    SimpleBVH::Ray r(origin, dir);

    std::vector<unsigned int> intersected_triangles;
    bvh.ray_intersection(r, intersected_triangles);

    unsigned int expected_intersections = 2;

    CHECK(intersected_triangles.size() == expected_intersections);
}