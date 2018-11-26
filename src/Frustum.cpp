
#include <Eigen/Eigen>

struct Frustum
{
  public:
	Frustum(Eigen::Matrix4f _pose, float _hfov, float _vfov, float _npDistance, float _fpDistance)
	{	
		mPose = _pose; // TODO: Constructor initialization
		mHFof = _hfov;
		mVFof = _vfov;
		mNpDistance = _npDistance;
		mFpDistance = _fpDistance;
	}

	// Frustum pose
	Eigen::Matrix4f mPose;

	// FOV rad
	float mHFof;
	float mVFof;

	// Plane distance
	float mNpDistance;
	float mFpDistance;
};