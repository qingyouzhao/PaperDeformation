#include "Vector.hh"
#include <vector>
#include <Eigen/Dense>
#include <algorithm>

template <class T>
class BezierCurve
{

	// A cubic bezier curve implementation
	Vector<T, 3> p0;
	Vector<T, 3> p1;
	Vector<T, 3> p2;
	Vector<T, 3> p3;

	
public:
	BezierCurve(const Vector<T, 3> InP0,const Vector<T, 3> InP1,const Vector<T, 3> InP2,const Vector<T, 3> InP3)
	:p0(InP0), p1(InP1), p2(InP2), p3(InP3),
	{}


	BezierCurve(const std::vector<Vector<T, 3>>& InPoints)
	{
		assert(InPoints.size() == 4);
		p0 = InPoints[0];
		p1 = InPoints[1];
		p2 = InPoints[2];
		p3 = InPoints[3];
		
	}


	// Evaluate the curve at time T
	Vector<T, 3> Eval(double InT);

	bool IsPointOnSpline(Vector<T, 3>& InPosition);
	// Calculate based on a piece wise evaluation of the point, should be more closely monitored.
	T    GetClosestPointOnSpline_Inaccurate(const Vector<T, 3>& InPosition, Vector<T,3>& OutPosition, T& OutT, int Segments = 10);
	bool IsPointOnSplineInaccurate(Vector<T, 3>& InPosition, int Segments);
};

template <class T>
T BezierCurve<T>::GetClosestPointOnSpline_Inaccurate(const Vector<T, 3>& InPosition, Vector<T, 3>& OutPosition, T& OutT, int Segments /*= 10*/)
{
	T distance = INT_MAX;
	T time = -1.0f;
	OutT = time;
	for (int i = 0; i < Segments; i++)
	{
		T t = double(i + 0.5) / Segments;
		Vector<T, 3> Pos = Eval(t);
		T d = length(Pos - InPosition);
		if ( d< distance)
		{
			distance = d;
			time = t;
			OutPosition = Pos;
			OutT = time;
		}
	}
	return time;
}

template <class T>
bool BezierCurve<T>::IsPointOnSplineInaccurate(Vector<T, 3>& InPosition, int Segments)
{
	Vector<T, 3> ClosestPosition; 
	T OutT;
	GetClosestPointOnSpline_Inaccurate(InPosition, ClosestPosition, OutT, Segments);
	T dist = length(ClosestPosition - InPosition);
	std::cout << "From " << InPosition.to_string() << std::endl; 
	std::cout << "Distance is " << dist << std::endl;
	if (dist< 1E-4)
	{
		return true;
	}
	return false;
}

template <class T>
bool BezierCurve<T>::IsPointOnSpline(Vector<T,3>& InPosition)
{
	//
	Eigen::Matrix<T, 3, 3> RealA;
	Eigen::Matrix<T, 3, 1> Realb;

	Vector<T, 3> T3_Coeff = (-p0 + (T)3*p1-(T)3*p2 + p3);
	Vector<T, 3> T2_Coeff = ((T)3*p0 - (T)6*p1 + (T)3* p2);
	Vector<T, 3> T_Coeff = - (T)3*p0 + (T)3*p1;

	RealA << T3_Coeff[0], T2_Coeff[0], T_Coeff[0],
			 T3_Coeff[1], T2_Coeff[1], T_Coeff[1], 
			 T3_Coeff[2], T2_Coeff[2], T_Coeff[2];
	Realb << (InPosition - p0)[0], (InPosition - p0)[1], (InPosition - p0)[2];
	Eigen::Matrix<T, 3, 1> T3T2T = RealA.colPivHouseholderQr().solve(Realb);

#ifndef _DEBUG
	std::cout << "Here is the Bezier matrix A:\n" << RealA << std::endl;
	std::cout << "Here is the Bezier vector b:\n" << Realb << std::endl;
	
	std::cout << "The solution is:\n" << T3T2T << std::endl;
#endif // DEBUG
	bool result = std::abs(T3T2T[0] - T3T2T[1] * T3T2T[2]) < 1e-4;
	return result;
}

template <class T>
Vector<T, 3> BezierCurve<T>::Eval(double InT)
{
	T clampedT = (T)std::min(std::max((double)InT, (double)0), (double)1);
	const T T2 = (T)(clampedT * clampedT);
	const T T3 = (T)(clampedT * T2);

	return 
			(	-	     p0 * T3 + (T)3 * p0 *T2 - (T)3 * p0 *clampedT + p0 +
				  (T)3 * p1 * T3 - (T)6 * p1 *T2 + (T)3 * p1 *clampedT
				- (T)3 * p2 * T3 + (T)3 * p2 *T2
				+	     p3 * T3);
}
