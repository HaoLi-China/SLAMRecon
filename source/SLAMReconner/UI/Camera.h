// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _CAMERA_H
#define _CAMERA_H

#include <Eigen/Geometry>

namespace Eigen{
class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Frame(const Eigen::Vector3f& pos = Eigen::Vector3f::Zero(),
                 const Eigen::Quaternionf& o = Eigen::Quaternionf::Identity())
        : orientation(o), position(pos)
    {}
    Frame lerp(float alpha, const Frame& other) const
    {
        return Frame((1.f-alpha)*position + alpha * other.position,
                     orientation.slerp(alpha,other.orientation));
    }

    Eigen::Quaternionf orientation;
    Eigen::Vector3f position;
};

class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Camera(void) : mViewIsUptodate(false), mProjIsUptodate(false), start(Vector3f::Zero()), delta(Vector3f::Zero())
    {
		mProjectionMatrix.setIdentity();
		mViewMatrix.setIdentity();

        mFovY = (float)M_PI / 5.0f;
		mNearDist = 0.1f;
        mFarDist = 100.0f;

		mVpX = 0;
		mVpY = 0;

		setPosition(Vector3f(1, 0, 0));
		setTarget(Vector3f(0,0,0));
    }

    Camera& operator=(const Camera& other)
    {
        mViewIsUptodate = false;
        mProjIsUptodate = false;

        mVpX = other.mVpX;
        mVpY = other.mVpY;
        mVpWidth = other.mVpWidth;
        mVpHeight = other.mVpHeight;

        mTarget = other.mTarget;
        mFovY = other.mFovY;
        mNearDist = other.mNearDist;
        mFarDist = other.mFarDist;

        mViewMatrix = other.mViewMatrix;
        mProjectionMatrix = other.mProjectionMatrix;

        return *this;
    }

    Camera(const Camera& other)
    {
        *this = other;
    }

    ~Camera()
    {
    }

	Eigen::Vector3f start, delta;

    void setViewport(unsigned int offsetx, unsigned int offsety, unsigned int width, unsigned int height)
    {
        mVpX = offsetx;
        mVpY = offsety;
        mVpWidth = width;
        mVpHeight = height;

        mProjIsUptodate = false;
    }
    void setViewport(unsigned int width, unsigned int height)
    {
        mVpWidth = width;
        mVpHeight = height;

        mProjIsUptodate = false;
    }

    inline unsigned int vpX(void) const { return mVpX; }
    inline unsigned int vpY(void) const { return mVpY; }
    inline unsigned int vpWidth(void) const { return mVpWidth; }
    inline unsigned int vpHeight(void) const { return mVpHeight; }

    inline float fovY(void) const { return mFovY; }
    void setFovY(float value)
    {
        mFovY = value;
        mProjIsUptodate = false;
    }

    void setPosition(const Eigen::Vector3f& pos)
    {
        mFrame.position = pos;
        mViewIsUptodate = false;
    }
    inline const Eigen::Vector3f& position(void) const { return mFrame.position; }

    void setOrientation(const Eigen::Quaternionf& q)
    {
        mFrame.orientation = q;
        mViewIsUptodate = false;
    }
    inline const Eigen::Quaternionf& orientation(void) const { 
		return mFrame.orientation; 
	}

    void setFrame(const Frame& f)
    {
        mFrame = f;
        mViewIsUptodate = false;
    }
    const Frame& frame(void) const { return mFrame; }

    void setDirection(const Eigen::Vector3f& newDirection)
    {
        Vector3f up = this->up();

        Matrix3f camAxes;

        camAxes.col(2) = (-newDirection).normalized();
        camAxes.col(0) = up.cross( camAxes.col(2) ).normalized();
        camAxes.col(1) = camAxes.col(2).cross( camAxes.col(0) ).normalized();
        setOrientation(Quaternionf(camAxes));

        mViewIsUptodate = false;
    }
    Eigen::Vector3f direction(void) const
    {
        return - (orientation() * Vector3f::UnitZ());
    }

    Eigen::Vector3f up(void) const
    {
        return orientation() * Vector3f::UnitZ();
    }
    Eigen::Vector3f right(void) const
    {
        return orientation() * Vector3f::UnitX();
    }

    void setTarget(const Eigen::Vector3f& target)
    {
        mTarget = target;
        if (!mTarget.isApprox(position()))
        {
			Vector3f newDirection = Vector3f(mTarget) - Vector3f(position());
            setDirection(newDirection.normalized());
        }
    }
    inline const Eigen::Vector3f& target(void) { return mTarget; }

    const Eigen::Affine3f& viewMatrix(void) const
    {
        updateViewMatrix();
        return mViewMatrix;
    }
    const Eigen::Matrix4f& projectionMatrix(void) const
    {
        updateProjectionMatrix();
        return mProjectionMatrix;
    }

    void rotateAroundTarget(const Eigen::Quaternionf& q)
    {
        Matrix4f mrot, mt, mtm;

        // update the transform matrix
        updateViewMatrix();
        Vector3f t = mViewMatrix * mTarget;

        mViewMatrix = Translation3f(t)
                * q
                * Translation3f(-t)
                * mViewMatrix;

        Quaternionf qa(mViewMatrix.linear());
        qa = qa.conjugate();
        setOrientation(qa);
        setPosition(- (qa * mViewMatrix.translation()) );

        mViewIsUptodate = true;
    }
    void localRotate(const Eigen::Quaternionf& q)
    {
        float dist = (position() - mTarget).norm();
        setOrientation(orientation() * q);
        mTarget = position() + dist * direction();
        mViewIsUptodate = false;
    }
    void zoom(float d)
    {
        float dist = (position() - mTarget).norm();
        if(dist > d)
        {
            setPosition(position() + direction() * d);
            mViewIsUptodate = false;
        }
    }

    void localTranslate(const Eigen::Vector3f& t)
    {
        Vector3f trans = orientation() * t;
        setPosition( position() + trans );
        setTarget( mTarget + trans );

        mViewIsUptodate = false;
    }

    Eigen::Vector3f unProject(const Eigen::Vector2f& uv, float depth, const Eigen::Matrix4f& invModelview) const
    {
        updateViewMatrix();
        updateProjectionMatrix();

        Vector3f a(2.*uv.x()/float(mVpWidth)-1., 2.*uv.y()/float(mVpHeight)-1., 1.);
        a.x() *= depth/mProjectionMatrix(0,0);
        a.y() *= depth/mProjectionMatrix(1,1);
        a.z() = -depth;
        // FIXME /\/|
        Vector4f b = invModelview * Vector4f(a.x(), a.y(), a.z(), 1.);
        return Vector3f(b.x(), b.y(), b.z());
    }
    Eigen::Vector3f unProject(const Eigen::Vector2f& uv, float depth) const
    {
        Matrix4f inv = mViewMatrix.inverse().matrix();
        return unProject(uv, depth, inv);
    }

protected:
    void updateViewMatrix(void) const
    {
        if(!mViewIsUptodate)
        {
            Quaternionf q = orientation().conjugate();
            mViewMatrix.linear() = q.toRotationMatrix();
            mViewMatrix.translation() = - (mViewMatrix.linear() * position());

            mViewIsUptodate = true;
        }
    }
    void updateProjectionMatrix(void) const
    {
        if(!mProjIsUptodate)
        {
            mProjectionMatrix.setIdentity();
            float aspect = float(mVpWidth)/float(mVpHeight);
            float theta = mFovY*0.5f;
            float range = mFarDist - mNearDist;
            float invtan = 1.0f/std::tan(theta);

            mProjectionMatrix(0,0) = invtan / aspect;
            mProjectionMatrix(1,1) = invtan;
            mProjectionMatrix(2,2) = -(mNearDist + mFarDist) / range;
            mProjectionMatrix(3,2) = -1;
            mProjectionMatrix(2,3) = -2 * mNearDist * mFarDist / range;
            mProjectionMatrix(3,3) = 0;

            mProjIsUptodate = true;
        }
    }

protected:

    unsigned int mVpX, mVpY;
    unsigned int mVpWidth, mVpHeight;

    Frame mFrame;

    mutable Eigen::Affine3f mViewMatrix;
    mutable Eigen::Matrix4f mProjectionMatrix;

    mutable bool mViewIsUptodate;
    mutable bool mProjIsUptodate;

    // used by rotateAroundTarget
    Eigen::Vector3f mTarget;

    float mFovY;
    float mNearDist;
    float mFarDist;
};

class Trackball
{
public:
	enum Mode { Around, Local };
	Trackball() : mpCamera(0) {}
	void start(Mode m = Around) { mMode = m; mLastPointOk = false; }
	void setCamera(Camera* pCam) { mpCamera = pCam; }
	void track(const Vector2i& point2D){
		if (mpCamera == 0) return;
		Vector3f newPoint3D;
		bool newPointOk = mapToSphere(point2D, newPoint3D);

		if (mLastPointOk && newPointOk)
		{
			Vector3f axis = mLastPoint3D.cross(newPoint3D).normalized();
			float cos_angle = mLastPoint3D.dot(newPoint3D);
			if (std::abs(cos_angle) < 1.0)
			{
				float angle = 2. * std::acos(cos_angle);
				if (mMode == Around)
					mpCamera->rotateAroundTarget(Quaternionf(AngleAxisf(angle, axis)));
				else
					mpCamera->localRotate(Quaternionf(AngleAxisf(-angle, axis)));
			}
		}

		mLastPoint3D = newPoint3D;
		mLastPointOk = newPointOk;
	}

protected:
	bool mapToSphere(const Vector2i& p2, Vector3f& v3)
	{
		if ((p2.x() >= 0) && (p2.x() <= int(mpCamera->vpWidth())) &&
			(p2.y() >= 0) && (p2.y() <= int(mpCamera->vpHeight())))
		{
			double x = (double)(p2.x() - 0.5*mpCamera->vpWidth()) / (double)mpCamera->vpWidth();
			double y = (double)(0.5*mpCamera->vpHeight() - p2.y()) / (double)mpCamera->vpHeight();
			double sinx = std::sin(M_PI * x * 0.5);
			double siny = std::sin(M_PI * y * 0.5);
			double sinx2siny2 = sinx * sinx + siny * siny;

			v3.x() = sinx;
			v3.y() = siny;
			v3.z() = sinx2siny2 < 1.0 ? std::sqrt(1.0 - sinx2siny2) : 0.0;

			return true;
		}
		else
			return false;
	}

	Camera* mpCamera;
	Eigen::Vector3f mLastPoint3D;
	Mode mMode;
	bool mLastPointOk;
};

class Plane : public Eigen::Hyperplane<float,3>{
public:
    Plane(Eigen::Vector3f Normal, Eigen::Vector3f Origin = Eigen::Vector3f::Zero()){
        normal() = Normal;
        offset() = -Origin.dot(Normal);
    }
};

} // namespace Eigen

#endif