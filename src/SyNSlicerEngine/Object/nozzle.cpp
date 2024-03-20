#include "nozzle.h"

using SyNSlicerEngine::Object::Nozzle;

Nozzle::Nozzle()
	: m_d(0.4)
	, m_x(10)
	, m_y(10)
{

}

Nozzle::Nozzle(const Nozzle &other)
{
	*this = other;
}

Nozzle::Nozzle(double d, double x, double y)
{
	this->setNozzle(d, x, y);
}

Nozzle::~Nozzle()
{

}

void Nozzle::setNozzle(double d, double x, double y)
{
	this->m_d = d;
	this->m_x = x;
	this->m_y = y;
}

double Nozzle::getD() const
{
	return m_d;
}

double Nozzle::getX() const
{
	return m_x;
}

double Nozzle::getY() const
{
	return m_y;
}

Nozzle &Nozzle::operator=(const Nozzle &other)
{
	this->m_d = other.m_d;
	this->m_x = other.m_x;
	this->m_y = other.m_y;
	return *this;
}
