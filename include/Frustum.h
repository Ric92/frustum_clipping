#ifndef FRUSTUM_H_
#define FRUSTUM_H_
#include <Eigen/Eigen>
#include <utility>

struct Frustum
{
	// _pose: Pose of Frustum
	// _hfov,_vfov: Vertical and horizontal FOV in degrees
	// _npDistance,fpDistance: Distance between pose and nearest and farest plane
	Frustum(int _id, Eigen::Matrix4f _pose, float _hfov, float _vfov, float _npDistance, float _fpDistance)
	{
		id = _id;
		mPose = _pose;
		mHFov = float(_vfov * M_PI / 180); // degrees to radians
		mVFov = float(_hfov * M_PI / 180); // degrees to radians

		mNpDistance = _npDistance;
		mFpDistance = _fpDistance;

		Eigen::Vector3f view = mPose.block(0, 0, 3, 1);  // view vector for the camera  - first column of the rotation matrix
		Eigen::Vector3f up = mPose.block(0, 1, 3, 1);	// up vector for the camera    - second column of the rotation matrix
		Eigen::Vector3f right = mPose.block(0, 2, 3, 1); // right vector for the camera - third column of the rotation matrix
		mPosition = mPose.block(0, 3, 3, 1);			 // Frustum position

		mNp_height = float(2 * tan(mVFov / 2) * mNpDistance);
		mNp_width = float(2 * tan(mHFov / 2) * mNpDistance);
		mFp_height = float(2 * tan(mVFov / 2) * mFpDistance);
		mFp_width = float(2 * tan(mHFov / 2) * mFpDistance);

		// 8 points of frustum
		mFpCenter = mPosition + view * mFpDistance;								   // far plane center
		mFpTopLeft = mFpCenter + (up * mFp_height / 2) - (right * mFp_width / 2);  // Top left corner of the far plane
		mFpTopRight = mFpCenter + (up * mFp_height / 2) + (right * mFp_width / 2); // Top right corner of the far plane
		mFpBotLeft = mFpCenter - (up * mFp_height / 2) - (right * mFp_width / 2);  // Bottom left corner of the far plane
		mFpBotRight = mFpCenter - (up * mFp_height / 2) + (right * mFp_width / 2); // Bottom right corner of the far plane

		mNpCenter = mPosition + view * mNpDistance;								   // near plane center
		mNpTopLeft = mNpCenter + (up * mNp_height / 2) - (right * mNp_width / 2);  // Top left corner of the near plane
		mNpTopRight = mNpCenter + (up * mNp_height / 2) + (right * mNp_width / 2); // Top right corner of the near plane
		mNpBotLeft = mNpCenter - (up * mNp_height / 2) - (right * mNp_width / 2);  // Bottom left corner of the near plane
		mNpBotRight = mNpCenter - (up * mNp_height / 2) + (right * mNp_width / 2); // Bottom right corner of the near plane

		// Plane eq: Ax + By + Cz + D = 0
		// Far plane
		mFplaneNormal = mFpCenter - mNpCenter;
		mFplaneNormal.normalize();
		mFplane.head(3) = mFplaneNormal;
		mFplane[3] = _fpDistance;

		// Near plane
		mNplaneNormal = mNpCenter - mFpCenter;
		mNplaneNormal.normalize();
		mNplane.head(3) = mNplaneNormal;
		mNplane[3] = _npDistance;

		// Up plane
		mUpPlaneNormal = (mNpTopLeft - mFpTopLeft).cross(mNpTopRight - mFpTopLeft);
		mUpPlaneNormal.normalize();
		mUpPlane.head(3) = mUpPlaneNormal;

		// Down plane
		mDownPlaneNormal = (mNpBotLeft - mFpBotLeft).cross(mNpBotRight - mFpBotLeft);
		mDownPlaneNormal.normalize();
		mDownPlane = -mUpPlane;

		// Right plane
		mRightPlaneNormal = -(mNpBotRight - mFpBotRight).cross(mNpTopRight - mFpBotRight);
		mRightPlaneNormal.normalize();
		mRightPlane.head(3) = mRightPlaneNormal;

		// Left plane
		mLeftPlaneNormal = -mRightPlaneNormal;
		mLeftPlaneNormal.normalize();
		mLeftPlane = -mRightPlane;

		// Near plane edges
		mEdges.push_back(std::make_pair(mNpBotLeft, mNpTopLeft));
		mEdges.push_back(std::make_pair(mNpTopLeft, mNpTopRight));
		mEdges.push_back(std::make_pair(mNpTopRight, mNpBotRight));
		mEdges.push_back(std::make_pair(mNpBotRight, mNpBotLeft));

		// Far plane edges
		mEdges.push_back(std::make_pair(mFpBotLeft, mFpTopLeft));
		mEdges.push_back(std::make_pair(mFpTopLeft, mFpTopRight));
		mEdges.push_back(std::make_pair(mFpTopRight, mFpBotRight));
		mEdges.push_back(std::make_pair(mFpBotRight, mFpBotLeft));

		// Connecting corners edges
		mEdges.push_back(std::make_pair(mNpBotLeft, mFpBotLeft));
		mEdges.push_back(std::make_pair(mNpTopLeft, mFpTopLeft));
		mEdges.push_back(std::make_pair(mNpTopRight, mFpTopRight));
		mEdges.push_back(std::make_pair(mNpBotRight, mFpBotRight));
	}

	int id;

	// Frustum pose
	Eigen::Matrix4f mPose;
	Eigen::Vector3f mPosition;
	// FOV rad
	float mHFov;
	float mVFov;

	// Plane distance
	float mNpDistance;
	float mFpDistance;

	// Edges
	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> mEdges;

	// Far plane
	float mFp_height;
	float mFp_width;
	Eigen::Vector3f mFpCenter;
	Eigen::Vector3f mFpTopLeft;
	Eigen::Vector3f mFpTopRight;
	Eigen::Vector3f mFpBotLeft;
	Eigen::Vector3f mFpBotRight;
	Eigen::Vector4f mFplane;
	Eigen::Vector3f mFplaneNormal;

	// Near plane
	float mNp_height;
	float mNp_width;
	Eigen::Vector3f mNpCenter;
	Eigen::Vector3f mNpTopLeft;
	Eigen::Vector3f mNpTopRight;
	Eigen::Vector3f mNpBotLeft;
	Eigen::Vector3f mNpBotRight;
	Eigen::Vector4f mNplane;
	Eigen::Vector3f mNplaneNormal;

	// Up plane
	Eigen::Vector4f mUpPlane;
	Eigen::Vector3f mUpPlaneNormal;

	// Down plane
	Eigen::Vector4f mDownPlane;
	Eigen::Vector3f mDownPlaneNormal;

	// Right plane
	Eigen::Vector4f mRightPlane;
	Eigen::Vector3f mRightPlaneNormal;

	// Left plane
	Eigen::Vector4f mLeftPlane;
	Eigen::Vector3f mLeftPlaneNormal;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif