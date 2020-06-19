#pragma once


#include <Discregrid/common.hpp>
#include <Eigen/Dense>

class CubicKernel
{
public:
	 Discregrid::Real getRadius() { return m_radius; }
	 void setRadius(Discregrid::Real val)
	{
		m_radius = val;
		const Discregrid::Real pi = static_cast<Discregrid::Real>(M_PI);

		const Discregrid::Real h3 = m_radius*m_radius*m_radius;
		m_k = 8.0 / (pi*h3);
		m_l = 48.0 / (pi*h3);
		m_W_zero = W(Discregrid::Vector3r::Zero());
	}

public:
	Discregrid::Real W(Discregrid::Vector3r const& r)
	{
		Discregrid::Real res = 0.0;
		const Discregrid::Real rl = r.norm();
		const Discregrid::Real q = rl/m_radius;
		if (q <= 1.0)
		{
			if (q <= 0.5)
			{
				const Discregrid::Real q2 = q*q;
				const Discregrid::Real q3 = q2*q;
				res = m_k * (6.0*q3-6.0*q2+1.0);
			}
			else
			{
				auto _1mq = 1.0 - q;
				res = m_k * (2.0*_1mq*_1mq*_1mq);
			}
		}
		return res;
	}

	Discregrid::Vector3r gradW(const Discregrid::Vector3r &r)
	{
		using namespace Eigen;
		Discregrid::Vector3r res;
		const Discregrid::Real rl = r.norm();
		const Discregrid::Real q = rl / m_radius;
		if (q <= 1.0)
		{
			if (rl > 1.0e-6)
			{
				const Discregrid::Vector3r gradq = r * ((Discregrid::Real) 1.0 / (rl*m_radius));
				if (q <= 0.5)
				{
					res = m_l*q*((Discregrid::Real) 3.0*q - (Discregrid::Real) 2.0)*gradq;
				}
				else
				{
					const Discregrid::Real factor = 1.0 - q;
					res = m_l*(-factor*factor)*gradq;
				}
			}
		}
		else
			res.setZero();

		return res;
	}

	Discregrid::Real W_zero()
	{
		return m_W_zero;
	}

private:
	Discregrid::Real m_radius;
	Discregrid::Real m_k;
	Discregrid::Real m_l;
	Discregrid::Real m_W_zero;
};
