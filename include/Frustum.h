#ifndef FRUSTUM_H_
#define FRUSTUM_H_
#include <Eigen/Eigen>
#include <utility>
#include "Facet.h"
#include <unordered_map>

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

		Eigen::Vector3f view = mPose.block(0, 0, 3, 1);  // view vector   
		Eigen::Vector3f up = mPose.block(0, 1, 3, 1);	// up vector     
		Eigen::Vector3f right = mPose.block(0, 2, 3, 1); // right vector 
		mPosition = mPose.block(0, 3, 3, 1);			 // Frustum position

		mNp_height = float(2 * tan(mVFov / 2) * mNpDistance);
		mNp_width = float(2 * tan(mHFov / 2) * mNpDistance);
		mFp_height = float(2 * tan(mVFov / 2) * mFpDistance);
		mFp_width = float(2 * tan(mHFov / 2) * mFpDistance);

		// far plane
		mFpCenter = mPosition + view * mFpDistance;								   
		mFpTopLeft = mFpCenter + (up * mFp_height / 2) - (right * mFp_width / 2);  
		mFpTopRight = mFpCenter + (up * mFp_height / 2) + (right * mFp_width / 2); 
		mFpBotLeft = mFpCenter - (up * mFp_height / 2) - (right * mFp_width / 2);  
		mFpBotRight = mFpCenter - (up * mFp_height / 2) + (right * mFp_width / 2); 

		// near plane
		mNpCenter = mPosition + view * mNpDistance;								   
		mNpTopLeft = mNpCenter + (up * mNp_height / 2) - (right * mNp_width / 2);  
		mNpTopRight = mNpCenter + (up * mNp_height / 2) + (right * mNp_width / 2); 
		mNpBotLeft = mNpCenter - (up * mNp_height / 2) - (right * mNp_width / 2);  
		mNpBotRight = mNpCenter - (up * mNp_height / 2) + (right * mNp_width / 2); 

		// Plane eq: Ax + By + Cz + D = 0
		// Far plane
		// mFplaneNormal = mFpCenter - mNpCenter;
		// mFplaneNormal.normalize();
		// mFplane.head(3) = mFplaneNormal;
		// mFplane[3] = _fpDistance;
		mFplaneNormal = (mFpTopRight-mFpTopLeft).cross(mFpBotRight-mFpTopLeft);
		mFplaneNormal.normalize();
		mFplane.head(3) = mFplaneNormal;
		mFplane[3] = mFpTopRight.dot(mFplaneNormal);
		std::vector<Eigen::Vector3f> farVertex = {mFpTopRight, mFpTopLeft, mFpBotLeft, mFpBotRight};
		std::shared_ptr<Facet> farFacet(new Facet(-mFplane, farVertex));
		mFacets["far"] = farFacet;

		// Near plane
		mNplaneNormal = mNpCenter - mFpCenter;
		mNplaneNormal.normalize();
		mNplane.head(3) = mNplaneNormal;
		mNplane[3] = _npDistance;
		std::vector<Eigen::Vector3f> nearVertex = {mNpTopRight, mNpTopLeft, mNpBotLeft, mNpBotRight};
		std::shared_ptr<Facet> nearFacet(new Facet(mNplane, nearVertex));
		mFacets["near"] = nearFacet;

		// Up plane
		mUpPlaneNormal = (mNpTopLeft - mFpTopLeft).cross(mNpTopRight - mFpTopLeft);
		mUpPlaneNormal.normalize();
		mUpPlane.head(3) = mUpPlaneNormal;
		mUpPlane[3] = 0;
		std::vector<Eigen::Vector3f> upVertex = {mNpTopRight, mNpTopLeft, mFpTopLeft, mFpTopRight};
		std::shared_ptr<Facet> upFacet(new Facet(mUpPlane, upVertex));
		mFacets["up"] = upFacet;

		// Down plane
		mDownPlaneNormal = -(mNpBotLeft - mFpBotLeft).cross(mNpBotRight - mFpBotLeft);
		mDownPlaneNormal.normalize();
		mDownPlane.head(3) = mDownPlaneNormal;
		mDownPlane[3] = 0;
		std::vector<Eigen::Vector3f> downVertex = {mNpBotRight, mNpBotLeft, mFpBotLeft, mFpBotRight};
		std::shared_ptr<Facet> downFacet(new Facet(mDownPlane, downVertex));
		mFacets["down"] = downFacet;

		// Right plane
		mRightPlaneNormal = -(mNpBotRight - mFpBotRight).cross(mNpTopRight - mFpBotRight);
		mRightPlaneNormal.normalize();
		mRightPlane.head(3) = mRightPlaneNormal;
		mRightPlane[3] = 0;
		std::vector<Eigen::Vector3f> rightVertex = {mNpBotRight, mNpTopRight, mFpTopRight, mFpBotRight};
		std::shared_ptr<Facet> rightFacet(new Facet(mRightPlane, rightVertex));
		mFacets["right"] = rightFacet;

		// Left plane
		mLeftPlaneNormal = (mNpBotLeft - mFpBotLeft).cross(mNpTopLeft - mFpBotLeft);
		mLeftPlaneNormal.normalize();
		mLeftPlane.head(3) = mLeftPlaneNormal;
		mLeftPlane[3] = 0;
		std::vector<Eigen::Vector3f> leftVertex = {mNpBotLeft, mNpTopLeft, mFpTopLeft, mFpBotLeft};
		std::shared_ptr<Facet> leftFacet(new Facet(mLeftPlane, leftVertex));
		mFacets["left"] = leftFacet;
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

	// Facets
	std::unordered_map<std::string, std::shared_ptr<Facet>> mFacets;

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