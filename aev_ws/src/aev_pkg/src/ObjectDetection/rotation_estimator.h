#ifndef ROTATION_ESTIMATOR_H
#define ROTATION_ESTIMATOR_H

#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include <cmath>

#ifndef PI
#define PI  3.14159265358979323846
#define PI_FL  3.141592f
#endif

struct float3 {
	float x, y, z;
	float3 operator*(float t)
	{
		return { x * t, y * t, z * t };
	}

	float3 operator-(float t)
	{
		return { x - t, y - t, z - t };
	}

	void operator*=(float t)
	{
		x = x * t;
		y = y * t;
		z = z * t;
	}

	void operator=(float3 other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
	}

	void add(float t1, float t2, float t3)
	{
		x += t1;
		y += t2;
		z += t3;
	}
};
struct float2 { float x, y; };

class rotation_estimator
{	
	// theta is the angle of camera rotation in x, y and z components
	float3 theta;
	std::mutex theta_mtx;
	/* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
	values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
	float alpha = 0.98f;
	bool firstGyro = true;
	bool firstAccel = true;
	// Keeps the arrival time of previous gyro frame
	double last_ts_gyro = 0;
public:
	rotation_estimator(void);
	// explicit
	rotation_estimator(rotation_estimator &&) {}
	// implicit
	rotation_estimator(const rotation_estimator&) = default;
	rotation_estimator& operator=(const rotation_estimator&) = default;
	~rotation_estimator();

	// Function to calculate the change in angle of motion based on data from gyro
	void process_gyro(rs2_vector gyro_data, double ts);

	void process_accel(rs2_vector accel_data);

	// Returns the current rotation angle
	float3 get_theta();
};

#endif // !ROTATION_ESTIMATOR_H