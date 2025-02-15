#pragma once

#include <Eigen/Core>

#include <vector>
#include <array>
#include <cassert>

namespace SimpleBVH {

using VectorMax3d =
    Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, 3, 1>;

struct Ray {
    Ray(Eigen::Vector3d O, Eigen::Vector3d D)
        : origin(O)
        , direction(D)
    {
    }

    Ray() { }
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
};

class BVH {
public:
    void init(const std::vector<std::array<Eigen::Vector3d, 2>>& cornerlist);

    void
    init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const double tol);

    void clear()
    {
        boxlist.clear();
        new2old.clear();
        n_corners = -1;
    }

    static bool ray_triangle_intersection(
        const Eigen::Vector3d& O,
        const Eigen::Vector3d& D,
        const Eigen::Vector3d& A,
        const Eigen::Vector3d& B,
        const Eigen::Vector3d& C,
        double& t,
        double& u,
        double& v,
        Eigen::Vector3d& N)
    {
        Eigen::Vector3d E1(B - A);
        Eigen::Vector3d E2(C - A);
        N = E1.cross(E2);
        double det = -D.dot(N);
        double invdet = 1.0 / det;
        Eigen::Vector3d AO = O - A;
        Eigen::Vector3d DAO = AO.cross(D);
        u = E2.dot(DAO) * invdet;
        v = -E1.dot(DAO) * invdet;
        t = AO.dot(N) * invdet;
        return (
            (fabs(det) >= 1e-20) && (t >= 0.0) && (u >= 0.0) && (v >= 0.0)
            && ((u + v) <= 1.0));
    }

    void intersect_3D_box(
        const Eigen::Vector3d& bbd0,
        const Eigen::Vector3d& bbd1,
        std::vector<unsigned int>& list) const
    {
        std::vector<unsigned int> tmp;
        assert(n_corners >= 0);
        box_search_recursive(bbd0, bbd1, tmp, 1, 0, n_corners);

        list.resize(tmp.size());
        for (int i = 0; i < tmp.size(); ++i)
            list[i] = new2old[tmp[i]];
    }

    void intersect_2D_box(
        const Eigen::Vector2d& bbd0,
        const Eigen::Vector2d& bbd1,
        std::vector<unsigned int>& list) const
    {
        Eigen::Vector3d bbd0_3D = Eigen::Vector3d::Zero();
        bbd0_3D.head<2>() = bbd0;

        Eigen::Vector3d bbd1_3D = Eigen::Vector3d::Zero();
        bbd1_3D.head<2>() = bbd1;

        intersect_3D_box(bbd0_3D, bbd1_3D, list);
    }

    void intersect_box(
        const VectorMax3d& bbd0,
        const VectorMax3d& bbd1,
        std::vector<unsigned int>& list) const
    {
        Eigen::Vector3d bbd0_3D = Eigen::Vector3d::Zero();
        bbd0_3D.head(bbd0.size()) = bbd0.head(bbd0.size());

        Eigen::Vector3d bbd1_3D = Eigen::Vector3d::Zero();
        bbd1_3D.head(bbd1.size()) = bbd1.head(bbd1.size());

        intersect_3D_box(bbd0_3D, bbd1_3D, list);
    }

    // tmax optional maximum parameter of the intersection along  the ray.
    // To do segement (between q1 and q2) intersection, call the function as
    // ray_intersection(Ray(q1, q2-q1), list, 1.0);
    void ray_intersection(
        const Ray& R,
        std::vector<unsigned int>& list,
        double tmax = std::numeric_limits<double>::max()) const
    {
        Eigen::Vector3d dirinv(
            1.0 / R.direction[0], 1.0 / R.direction[1], 1.0 / R.direction[2]);
        ray_intersection_recursive(R, dirinv, tmax, 1, 0, n_corners, list);

        for (int i = 0; i < list.size(); ++i) {
            list[i] = new2old[list[i]];
        }
    }

private:
    void init_boxes_recursive(
        const std::vector<std::array<Eigen::Vector3d, 2>>& cornerlist,
        int node_index,
        int b,
        int e);

    void box_search_recursive(
        const Eigen::Vector3d& bbd0,
        const Eigen::Vector3d& bbd1,
        std::vector<unsigned int>& list,
        int n,
        int b,
        int e) const;

    bool box_intersects_box(
        const Eigen::Vector3d& bbd0,
        const Eigen::Vector3d& bbd1,
        int index) const;

    void BVH::ray_intersection_recursive(
        const Ray& R,
        const Eigen::Vector3d& dirinv,
        double tmax,
        size_t n,
        size_t b,
        size_t e,
        std::vector<unsigned int>& list) const;

    static int max_node_index(int node_index, int b, int e);

    std::vector<std::array<Eigen::Vector3d, 2>> boxlist;
    std::vector<int> new2old;
    size_t n_corners = -1;
};
} // namespace SimpleBVH
