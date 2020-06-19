#pragma once

#include <Discregrid/common.hpp>
#include <Eigen/Dense>

class GaussQuadrature
{
public:

    using Integrand = std::function<Discregrid::Real(Discregrid::Vector3r const&)>;
    using Domain = Discregrid::AlignedBox3r;

    static Discregrid::Real integrate(Integrand integrand, Domain const& domain, unsigned int p);
};



