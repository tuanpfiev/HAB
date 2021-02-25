#pragma once

/* Don't worry, I'm aware just how barren this is compared to what it should be based on this-|
																							  |
This is the hmath Hamiltonian Mathematics Library											  |
																							  |
				  █████          ██															  |
			   ██████  █        ████ █														  |
			  ██   █  █         █████														  |
			 █    █  █          █ █															  |
				 █  █           █															  |
				██ ██           █															  |
				██ ██           █															  |
				██ ██████████████															  |
				██ ██           █															  |
				██ ██           ██															  |
				█  ██           ██															  |
				   █             ██															  |
			   ████              ██															  |
			  █  █████            ██														  |
			 █     ██	       																  |
			 █			       																  |
			  █			       																  |
			   ██		       																  |
																							  |
This library encompasses vectors in 2d and 3d					 <----------------------------|
euclidean spaces, complex numbers, dual 3d vectors,				 <----------------------------|
quaternions, and dual quaternions.								 <----------------------------|
																 <----------------------------|
Some of these algebras are rather niche/complicated				 <----------------------------|
so I suggest you look at the wiki for reference on 				 <----------------------------|
the algebras and their implementation.							 <----------------------------|
																 <----------------------------|
For the dual algebras, I suggest you read Multi-Body			 <----------------------------|
Kinematics and Dynamics with Lie Groups by Chevallier			 <----------------------------|
Lerbet. This is quite a nice read, though quite technical.		 <----------------------------|
																 <----------------------------|
For the quaternions and complex numbers, I suggest Quaternions	 <----------------------------|
and Rotation Sequences by Jack B. Kuipers. This is another		 <----------------------------|
great read.

Please excuse the ascii art...it looks good on Visual Studio but nowhere else :(
*/

#include <cmath>
#include <iostream>

namespace hmath
{
	// constant expression mathematical constants

	constexpr long double H_PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170680;
	constexpr long double H_PI_2 = 1.5707963267948966192313216916397514420985846996875529104874722961539082031431044993140174126710585340;
	constexpr long double H_TAU = 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696506842341360;
	constexpr long double H_PI_squared = 9.8696044010893586188344909998761511353136994072407906264133493762200448224192052430017734037185522318;
	constexpr long double H_root_PI = 1.7724538509055160272981674833411451827975494561223871282138077898529112845910321813749506567385446654;
	constexpr long double H_e = 2.7182818284590452353602874713526624977572470936999595749669676277240766303535475945713821785251664274;
	constexpr long double H_e_2 = 1.3591409142295226176801437356763312488786235468499797874834838138620383151767737972856910892625832137;
	constexpr long double H_2_e = 5.4365636569180904707205749427053249955144941873999191499339352554481532607070951891427643570503328549;
	constexpr long double H_e_squared = 7.3890560989306502272304274605750078131803155705518473240871278225225737960790577633843124850791217948;
	constexpr long double H_root_e = 1.6487212707001281468486507878141635716537761007101480115750793116406610211942156086327765200563666430;

	// constant expressions for use in object construction

	typedef int HMATH_MAKE;

	constexpr HMATH_MAKE ELEMENT_WISE = 0;
	constexpr HMATH_MAKE VECTOR_TO = 1;
	constexpr HMATH_MAKE VECTOR3D_CROSS = 2;
	constexpr HMATH_MAKE NORMED = 3;
	constexpr HMATH_MAKE VECTOR_WISE = 4;
	constexpr HMATH_MAKE RADIANS = 5;
	constexpr HMATH_MAKE DEGREES = 6;
	constexpr HMATH_MAKE LINEAR_COMBINATION = 7;

	/* ############################## MISC FUNCTIONS ############################## */

	double toRad(double degrees)
	{
		return degrees * (H_PI / 180);
	}

	double toDegree(double rad)
	{
		return rad * (180 / H_PI);
	}

	bool double_equality(double d1, double d2, double epsilon)
	{
		return std::abs(d1 - d2) < epsilon;
	}

	double roundN(double val, int decimal_place)
	{
		val = round( val * (pow(10, decimal_place)) ) / (pow(10, decimal_place));
		return val;
	}

	/* ############################## ALGEBRA CLASSES ############################## */

	//class Complex
	//{
	//public:
	//	double a, b;

	//	Complex();
	//	~Complex();

	//private:

	//};

	/* ############################## VECTOR2 ############################## */

	class Vector2
	{
	public:
		double i, j;

		Vector2() {}

		Vector2(double i, double j, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->i = i;
				this->j = j;
			}
			else if (construct == NORMED)
			{
				double mag = sqrt(i * i + j * j);
				this->i = i / mag;
				this->j = j / mag;
			}
		}

		Vector2(hmath::Vector2 v1, hmath::Vector2 v2, HMATH_MAKE construct = VECTOR_TO)
		{
			if (construct == VECTOR_TO)
			{
				this->i = v2.i - v1.i;
				this->j = v2.j - v1.j;
			}
		}

		double norm()
		{
			return sqrt((this->i * this->i) + (this->j * this->j));
		}

		Vector2 normed()
		{
			hmath::Vector2 new_vector;
			new_vector.i = this->i / this->norm();
			new_vector.j = this->j / this->norm();
			return new_vector;
		}
	};

	double dot(Vector2 v1, Vector2 v2)
	{
		return v1.i * v2.i + v1.j * v2.j;
	}

	std::ostream& operator<<(std::ostream& os, Vector2 v)
	{
		os << v.i << ", " << v.j;
		return os;
	}

	// Vector2 operator+(Vector2& v1, Vector2& v2)
	// {
	// 	return Vector2(v1.i + v2.i, v1.j + v2.j);
	// }

	Vector2 operator+(Vector2 v1, Vector2 v2)
	{
		return Vector2(v1.i + v2.i, v1.j + v2.j);
	}

	void operator+=(Vector2& v1, Vector2 v2)
	{
		v1.i += v2.i;
		v1.j += v2.j;
	}

	Vector2 operator-(Vector2 v1, Vector2 v2)
	{
		return Vector2(v1.i - v2.i, v1.j - v2.j);
	}

	void operator-=(Vector2& v1, Vector2 v2)
	{
		v1.i -= v2.i;
		v1.j -= v2.j;
	}

	Vector2 operator*(Vector2 v, double d)
	{
		return Vector2(v.i * d, v.j * d);
	}

	/* ############################## VECTOR3 ############################## */

	class Vector3
	{
	public:
		double i, j, k;

		Vector3() {}

		Vector3(double i, double j, double k, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->i = i;
				this->j = j;
				this->k = k;
			}
			else if (construct == NORMED)
			{
				double mag = sqrt(i * i + j * j + k * k);
				this->i = i / mag;
				this->j = j / mag;
				this->k = k / mag;
			}
		}

		Vector3(hmath::Vector3 v1, hmath::Vector3 v2, HMATH_MAKE construct = VECTOR_TO)
		{
			if (construct == VECTOR_TO)
			{
				this->i = v2.i - v1.i;
				this->j = v2.j - v1.j;
				this->k = v2.k - v1.k;
			}
			else if (construct == VECTOR3D_CROSS)
			{
				this->i = v1.j * v2.k - v1.k * v2.j;
				this->j = v1.k * v2.i - v1.i * v2.k;
				this->k = v1.i * v2.j - v1.j * v2.i;
			}
		}

		Vector3(Vector2 v)
		{
			this->i = v.i;
			this->j = v.j;
			this->k = 0;
		}

		Vector3 normed()
		{
			hmath::Vector3 new_vector;
			new_vector.i = this->i / this->norm();
			new_vector.j = this->j / this->norm();
			new_vector.k = this->k / this->norm();
			return new_vector;
		}

		double norm()
		{
			return sqrt((this->i * this->i) + (this->j * this->j) + (this->k * this->k));
		}

		void normalize()
		{
			double mag = this->norm();

			this->i *= 1 / mag;
			this->j *= 1 / mag;
			this->k *= 1 / mag;
		}		
	};

	double dot(Vector3 v1, Vector3 v2)
	{
		return v1.i * v2.i + v1.j * v2.j + v1.k * v2.k;
	}

	std::ostream& operator<<(std::ostream& os, Vector3& v)
	{
		os << v.i << ", " << v.j << ", " << v.k;
		return os;
	}

	Vector3 operator+(Vector3& v1, Vector3& v2)
	{
		return Vector3(v1.i + v2.i, v1.j + v2.j, v1.k + v2.k);
	}

	void operator+=(Vector3& v1, Vector3 v2)
	{
		v1.i += v2.i;
		v1.j += v2.j;
		v1.k += v2.k;
	}

	Vector3 operator-(Vector3& v1, Vector3& v2)
	{
		return Vector3(v1.i - v2.i, v1.j - v2.j, v1.k - v2.k);
	}

	void operator-=(Vector3& v1, Vector3& v2)
	{
		v1.i -= v2.i;
		v1.j -= v2.j;
		v1.k -= v2.k;
	}

	Vector3 operator*(Vector3 v, double scalar)
	{
		return Vector3(v.i * scalar, v.j * scalar, v.k * scalar);
	}

	void operator*=(Vector3& v, double scalar)
	{
		v.i *= scalar;
		v.j *= scalar;
		v.k *= scalar;
	}

	/* ############################## LONG VECTOR3 ############################### */

	class lVector3
	{
	public:
		long double i, j, k;

		lVector3() {}

		lVector3(long double i, long double j, long double k, HMATH_MAKE construct = ELEMENT_WISE)
		{
			if (construct == ELEMENT_WISE)
			{
				this->i = i;
				this->j = j;
				this->k = k;
			}
			else if (construct == NORMED)
			{
				long double mag = sqrt(i * i + j * j + k * k);
				this->i = i / mag;
				this->j = j / mag;
				this->k = k / mag;
			}
		}

		lVector3(hmath::lVector3 v1, hmath::lVector3 v2, HMATH_MAKE construct = VECTOR_TO)
		{
			if (construct == VECTOR_TO)
			{
				this->i = v2.i - v1.i;
				this->j = v2.j - v1.j;
				this->k = v2.k - v1.k;
			}
			else if (construct == VECTOR3D_CROSS)
			{
				this->i = v1.j * v2.k - v1.k * v2.j;
				this->j = v1.k * v2.i - v1.i * v2.k;
				this->k = v1.i * v2.j - v1.j * v2.i;
			}
		}

		lVector3(Vector2 v)
		{
			this->i = v.i;
			this->j = v.j;
			this->k = 0;
		}

		lVector3 normed()
		{
			hmath::lVector3 new_vector;
			new_vector.i = this->i / this->norm();
			new_vector.j = this->j / this->norm();
			new_vector.k = this->k / this->norm();
			return new_vector;
		}

		long double norm()
		{
			return sqrt((this->i * this->i) + (this->j * this->j) + (this->k * this->k));
		}

		void normalize()
		{
			long double mag = this->norm();

			this->i *= 1 / mag;
			this->j *= 1 / mag;
			this->k *= 1 / mag;
		}		
	};

	long double dot(lVector3 v1, lVector3 v2)
	{
		return v1.i * v2.i + v1.j * v2.j + v1.k * v2.k;
	}

	std::ostream& operator<<(std::ostream& os, lVector3& v)
	{
		os << v.i << ", " << v.j << ", " << v.k;
		return os;
	}

	lVector3 operator+(lVector3& v1, lVector3& v2)
	{
		return lVector3(v1.i + v2.i, v1.j + v2.j, v1.k + v2.k);
	}

	void operator+=(lVector3& v1, lVector3 v2)
	{
		v1.i += v2.i;
		v1.j += v2.j;
		v1.k += v2.k;
	}

	lVector3 operator-(lVector3& v1, lVector3& v2)
	{
		return lVector3(v1.i - v2.i, v1.j - v2.j, v1.k - v2.k);
	}

	void operator-=(lVector3& v1, lVector3& v2)
	{
		v1.i -= v2.i;
		v1.j -= v2.j;
		v1.k -= v2.k;
	}

	lVector3 operator*(lVector3 v, long double scalar)
	{
		return lVector3(v.i * scalar, v.j * scalar, v.k * scalar);
	}

	void operator*=(lVector3& v, long double scalar)
	{
		v.i *= scalar;
		v.j *= scalar;
		v.k *= scalar;
	}

	/* ############################## SPHERICAL COORDINATE ############################## */

	class Spherical
	{
	public:
		double theta, phi, r;

		Spherical() {};

		Spherical(double theta, double phi, double r, HMATH_MAKE construct = RADIANS)
		{
			if (construct == RADIANS)
			{
				this->theta = theta;
				this->phi = phi;
				this->r = r;
			}
			else if (construct == DEGREES)
			{
				this->theta = theta * (H_PI / 180);
				this->phi = phi * (H_PI / 180);
				this->r = r;
			}
		}

		Spherical(Vector3 v)
		{
			this->theta = atan2(v.normed().j, v.normed().i);
			this->phi = asin(v.normed().k);
			this->r = v.norm();
		}

		Vector3 R3()
		{
			Vector3 v;
			v.i = this->r * cos(this->phi) * cos(this->theta);
			v.j = this->r * cos(this->phi) * sin(this->theta);
			v.k = this->r * sin(this->phi);
			return v;
		}

		std::string degreeString()
		{
			std::string return_string = std::to_string(toDegree(this->theta)) + ", " + std::to_string(toDegree(this->phi)) + ", " + std::to_string(this->r);
			return return_string;
		}
	};

	std::ostream& operator<<(std::ostream& os, Spherical& s)
	{
		os << s.theta << ", " << s.phi << ", " << s.r;
		return os;
	}
}
