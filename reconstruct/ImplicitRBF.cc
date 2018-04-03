//=============================================================================

#include "ImplicitRBF.hh"

//== IMPLEMENTATION ==========================================================

ImplicitRBF::ImplicitRBF( 
	const std::vector<Vec3f>& _points, 
	const std::vector<Vec3f>& _normals )
{
	//////////////////////////////////////////////////////////////////////
	// INSERT CODE:
	// Set up necessary parameters


	const int n = _points.size();
	Vec3d Max((double)(INT_MIN), (double)(INT_MIN), (double)(INT_MIN));
	Vec3d Min((double)(INT_MAX), (double)(INT_MAX), (double)(INT_MAX));
	for (int i = 0; i < n; i++)
	{
		if (_points[i][0] < Min[0]) Min[0] = _points[i][0];
		if (_points[i][1] < Min[1]) Min[1] = _points[i][1];
		if (_points[i][2] < Min[2]) Min[2] = _points[i][2];
		if (_points[i][0] > Max[0]) Max[0] = _points[i][0];
		if (_points[i][1] > Max[1]) Max[1] = _points[i][1];
		if (_points[i][2] > Max[2]) Max[2] = _points[i][2];
	}
	
	gmmMatrix M_phi(2*n,2*n);
	gmmVector d(2*n);
	weights_.resize(2*n);
	centers_.resize(2*n);
	double epsilon = (Max - Min).norm() * 0.01;
	for (auto a : M_phi)
	{
		a = 0;
	}
	for(auto a:weights_)
	{
		a = 0;
	}

	// 1) collect constraints (on-surface and off-surface)
	// On surface 
	const bool bUseIJ = true;
	for (int i = 0; i < n; i++)
	{
		centers_[i] = _points[i];
		centers_[n + i] = _points[i] + epsilon * _normals[i];
		d[i] = 0;
		d[n + i] = epsilon;
	}
	if(bUseIJ)
	{
		for (int i = 0; i < n; i++) // index for row
		{
			for (int j = 0; j < 2 * n; j++) // index for column, iterate ith x over all centers
			{
				// ci spans from 0...i			// xi is just xj
				double Phi_ij = ImplicitRBF::kernel(centers_[j], (Vec3d)_points[i]);
				M_phi(i, j) = Phi_ij;
			}
		}

		// Off surface
		for (int i = 0; i < n; i++) // index for row
		{
			for (int j = 0; j < 2 * n; j++) // index for column, iterate ith x over all centers
			{
				double Phi_ij = ImplicitRBF::kernel(centers_[j], (Vec3d)(_points[i] + epsilon * _normals[i]));
				M_phi(n + i, j) = Phi_ij;
			}
		}

	}
	else
	{
		for (int i = 0; i < n; i++) // index for row
		{
			for (int j = 0; j < n; j++) // index for column, iterate ith x over all centers
			{
				// ci spans from 0...i			// xi is just xj
				double Phi_ij = ImplicitRBF::kernel(_points[i], _points[j]);
				M_phi(i, j) = Phi_ij;
			}
			d[i] = 0;
			centers_[i] = _points[i];
		}

		// Off surface
		for (int i = 0; i < n; i++) // index for row
		{
			for (int j = 0; j < n; j++) // index for column, iterate ith x over all centers
			{
				double Phi_ij = ImplicitRBF::kernel(_points[i], _points[j] + epsilon * _normals[j]);
				M_phi(n + i, n + j) = Phi_ij;
			}
			d[n + i] = epsilon;
			centers_[n + i] = _points[i];
		}

	}// 2) setup matrix

	// 3) solve linear system for weights_
	solve_linear_system(M_phi, d, weights_);
	// Weight n到weight 2n 应该是一样的啊
	//////////////////////////////////////////////////////////////////////
}

//-----------------------------------------------------------------------------

void ImplicitRBF::solve_linear_system( 
	gmmMatrix& _M, 
	gmmVector& _b, 
	gmmVector& _x )
{
	// solve linear system by gmm's LU factorization
	unsigned int N = _b.size();
	_x.resize(N);
	std::vector< size_t >  ipvt(N);
	gmm::lu_factor( _M, ipvt );
	gmm::lu_solve( _M, ipvt, _x, _b );
}

//-----------------------------------------------------------------------------

double ImplicitRBF::operator()(const Vec3f& _p) const
{
	std::vector<Vec3d>::const_iterator  
		c_it(centers_.begin()),
		c_end(centers_.end());

	std::vector<double>::const_iterator   
		w_it(weights_.begin());

	const Vec3d p(_p);
	double f(0);

	for (; c_it!=c_end; ++c_it, ++w_it)
		f += *w_it * kernel(*c_it, p);

	return f;
}

//=============================================================================
