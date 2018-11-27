
#include <Eigen/Eigen>

struct Frustum
{
	// _pose: Pose of Frustum
	// _hfov,_vfov: Vertical and horizontal FOV in degrees
	// _npDistance,fpDistance: Distance between pose and nearest and farest plane
	Frustum(Eigen::Matrix4f _pose, float _hfov, float _vfov, float _npDistance, float _fpDistance)
	{
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

		// Far plane eq
		mFplane.block(0, 0, 3, 1).matrix() = (mFpBotLeft - mFpBotRight).cross(mFpTopRight - mFpBotRight);
		mFplane(3) = -mFpCenter.dot(mFplane.block(0, 0, 3, 1));

		// Near plane eq
		mNplane.block(0, 0, 3, 1).matrix() = (mNpBotLeft - mNpBotRight).cross(mNpTopRight - mNpBotRight);
		mFplane(3) = -mNpCenter.dot(mNplane.block(0, 0, 3, 1));

		// Near plane eq
		Eigen::Vector3f q,v;
		q[0] =  mFpTopLeft[0] - mFpTopRight[0]; v[0] =  mFpTopLeft[0] - mFpBotLeft[0];
		q[1] =  mFpTopLeft[1] - mFpTopRight[1]; v[1] =  mFpTopLeft[1] - mFpBotLeft[1];
		q[2] =  mFpTopLeft[2] - mFpTopRight[2]; v[2] =  mFpTopLeft[2] - mFpBotLeft[2];
		mFpNormal = q.cross(v);
		mFpNormal.normalize();

		// Near plane eq
		Eigen::Vector3f p,u;
		p[0] =  mNpTopLeft[0] - mNpTopRight[0]; u[0] =  mNpTopLeft[0] - mNpBotLeft[0];
		p[1] =  mNpTopLeft[1] - mNpTopRight[1]; u[1] =  mNpTopLeft[1] - mNpBotLeft[1];
		p[2] =  mNpTopLeft[2] - mNpTopRight[2]; u[2] =  mNpTopLeft[2] - mNpBotLeft[2];
		mNpNormal = p.cross(u);
		mNpNormal.normalize();

		std::cout << "Plane vector3f: " << mNpNormal << "\n";
		std::cout << "Plane vector4f: " << mNplane << "\n";
	}

	// Frustum pose
	Eigen::Matrix4f mPose;
	Eigen::Vector3f mPosition;
	// FOV rad
	float mHFov;
	float mVFov;

	// Plane distance
	float mNpDistance;
	float mFpDistance;

	// Far plane
	float mFp_height;
	float mFp_width;
	Eigen::Vector3f mFpCenter;
	Eigen::Vector3f mFpTopLeft;
	Eigen::Vector3f mFpTopRight;
	Eigen::Vector3f mFpBotLeft;
	Eigen::Vector3f mFpBotRight;
	Eigen::Vector3f mFpNormal;
	Eigen::Vector4f mFplane;

	// Near plane
	float mNp_height;
	float mNp_width;
	Eigen::Vector3f mNpCenter;
	Eigen::Vector3f mNpTopLeft;
	Eigen::Vector3f mNpTopRight;
	Eigen::Vector3f mNpBotLeft;
	Eigen::Vector3f mNpBotRight;
	Eigen::Vector3f mNpNormal;
	Eigen::Vector4f mNplane;
};