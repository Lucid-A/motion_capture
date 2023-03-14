// ***********************************************************************
// Assembly         : 
// Author           : Administrator
// Created          : 08-05-2021
//
// Last Modified By : Administrator
// Last Modified On : 08-05-2021
// ***********************************************************************
// <copyright file="Utility.h" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>???????????????????????????????????</summary>
// ***********************************************************************

#pragma once

#include <list>
#include <fstream>
#include <string>
#include <cmath>

#define ERRORDATA	(9999999.999999) 
#define Mid(X) int(X/2)
#define FrameFactor 3  // 3,5,7
#define FPS 60

#define IN                                               //???																			
#define OUT                                              //????																						
#define IN_OUT                                           //?????

struct Point
{
	double x;
	double y;
	double z;
	long long TimeStamp;
	std::string name;
};

struct angle
{
	double heading;
	long long TimeStamp;
};

struct Vel      //???????
{
	double Vx;
	double Vy;
	double Vz;
	double Vr;// |V|
	Vel() :Vx(0), Vy(0), Vz(0), Vr(0) {}

	friend std::ostream& operator << (std::ostream& os, const Vel& record)
	{
		os << "Vx:" << record.Vx << "\t";
		os << "Vy:" << record.Vy << "\t";
		os << "Vz:" << record.Vz << "\t";
		os << "Vr:" << record.Vr << "\t";
		os << std::endl;

		return os;
	}
};

struct Accel      //????????
{
	double Ax;
	double Ay;
	double Az;
	double Ar; // |A|
	Accel() :Ax(0), Ay(0), Az(0), Ar(0) {}

	friend std::ostream& operator << (std::ostream& os, const Accel& record)
	{
		os << "Ax:" << record.Ax << "\t";
		os << "Ay:" << record.Ay << "\t";
		os << "Az:" << record.Az << "\t";
		os << "Ar:" << record.Ar << "\t";
		os << std::endl;

		return os;
	}
};

// ??????????????????????????????????
template<class T>
class CalculateMethod
{
public:
	typedef T ValueType;
	typedef T& ReferenceType;

	CalculateMethod(double FR, int FF) :m_Points(nullptr), m_FPS(FR), m_FrameFactor(FF) {};

	int GetFrameFactor() const { return m_FrameFactor; };
	int GetFrameFPS() const { return m_FPS; };

	int tryToCalculate(IN Point* point, OUT ReferenceType accel)
	{
		if (nullptr == point)
			return 1;

		m_Points = point;

		if (m_FPS <= 0 || m_FPS >= 400)
			return 1;

		return calculate(accel);
	}

protected:
	virtual int calculate(OUT ReferenceType accel) = 0;

protected:
	Point* m_Points;
	double m_FPS;
	int m_FrameFactor;
};

class CalculateVelocity : public CalculateMethod<Vel>
{
public:
	CalculateVelocity(double FR, int FF) :CalculateMethod(FR, FF) {};

protected:
	virtual int calculate(OUT Vel& vel) override
	{
		int TR = m_FrameFactor / 2;

		vel.Vx = 1000 * (m_Points[TR * 2].x - m_Points[0].x) / (m_Points[TR * 2].TimeStamp - m_Points[0].TimeStamp);
		vel.Vy = 1000 * (m_Points[TR * 2].y - m_Points[0].y) / (m_Points[TR * 2].TimeStamp - m_Points[0].TimeStamp);
		vel.Vz = 1000 * (m_Points[TR * 2].z - m_Points[0].z) / (m_Points[TR * 2].TimeStamp - m_Points[0].TimeStamp);
		vel.Vr = sqrt(vel.Vx * vel.Vx + vel.Vy * vel.Vy + vel.Vz * vel.Vz);

		return 0;
	}
};

// ??????????????????????
class CalculateVelocityByTwoFrame : public CalculateMethod<Vel>
{
public:
	CalculateVelocityByTwoFrame(double FR) :CalculateMethod(FR, 2) {};

protected:
	virtual int calculate(OUT Vel& vel) override
	{
		vel.Vx = 1000 * (m_Points[1].x - m_Points[0].x)/(m_Points[1].TimeStamp - m_Points[0].TimeStamp);
		vel.Vy = 1000 * (m_Points[1].y - m_Points[0].y)/(m_Points[1].TimeStamp - m_Points[0].TimeStamp);
		vel.Vz = 1000 * (m_Points[1].z - m_Points[0].z)/(m_Points[1].TimeStamp - m_Points[0].TimeStamp);
		vel.Vr = sqrt(vel.Vx * vel.Vx + vel.Vy * vel.Vy + vel.Vz * vel.Vz);

		return 0;
	}
};

class CalculateAcceleration : public CalculateMethod<Accel>
{
public:
	CalculateAcceleration(double FR, int FF) :CalculateMethod(FR, FF) {};
protected:
	virtual int calculate(OUT Accel& accel) override
	{
		int TR = m_FrameFactor / 2;

		accel.Ax = 1000 * 1000 * ((m_Points[TR * 2].x - m_Points[TR].x)/(m_Points[TR * 2].TimeStamp - m_Points[TR].TimeStamp) 
		                         -(m_Points[TR].x- m_Points[0].x)/(m_Points[TR].TimeStamp- m_Points[0].TimeStamp)) /
								  ((m_Points[TR * 2].TimeStamp - m_Points[0].TimeStamp)/2);
		accel.Ay = 1000 * 1000 * ((m_Points[TR * 2].y - m_Points[TR].y)/(m_Points[TR * 2].TimeStamp - m_Points[TR].TimeStamp) 
		                         -(m_Points[TR].y- m_Points[0].y)/(m_Points[TR].TimeStamp- m_Points[0].TimeStamp)) /
								  ((m_Points[TR * 2].TimeStamp - m_Points[0].TimeStamp)/2);
		accel.Az = 1000 * 1000 * ((m_Points[TR * 2].z - m_Points[TR].z)/(m_Points[TR * 2].TimeStamp - m_Points[TR].TimeStamp) 
		                         -(m_Points[TR].z- m_Points[0].z)/(m_Points[TR].TimeStamp- m_Points[0].TimeStamp)) /
								  ((m_Points[TR * 2].TimeStamp - m_Points[0].TimeStamp)/2);
		accel.Ar = sqrt(accel.Ax * accel.Ax + accel.Ay * accel.Ay + accel.Az * accel.Az);

		return 0;
	}
};

// ????????ï…?›¥?????????????????????????
class SlideFrameArray
{
public:

	explicit SlideFrameArray()
	{
		clear();
	}

	void clear()
	{
		_list.clear();
	}

	size_t cache(const Point& point)
	{
		_list.push_back(point);
		return _list.size();
	}

	size_t Cache(double x, double y, double z,long long TimeStamp)
	{
		Point point = { 0 };
		point.x = x;
		point.y = y;
		point.z = z;
        point.TimeStamp = TimeStamp;
		return cache(point);
	}

	template<class T>
	bool tryToCalculate(OUT T& data, CalculateMethod<T>& method)
	{
		Point* inArray = nullptr;
		if (!frameArray(inArray, method.GetFrameFactor()))
		{
			if (inArray)
			{
				delete[] inArray;
				inArray = nullptr;
			}

			return false;
		}

		int retCount = method.tryToCalculate(inArray, data);
		if (retCount != 0)
		{
			delete[] inArray;
			return false;
		}

		delete[] inArray;
		return true;
	}

private:
	// the retArray need to be free by the user

	bool frameArray(OUT Point*& retArray, int frameFactor = FrameFactor)
	{
		if (_list.size() < frameFactor)
			return false;

		retArray = new Point[frameFactor];

		int index = 0;
		for (auto itor = _list.begin(); itor != _list.end() && index < frameFactor; ++itor, ++index)
		{
			retArray[index] = *itor;
		}

		// ???????
		_list.pop_front();

		return true;
	}

private:
	std::list<Point> _list;
};