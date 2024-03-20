#ifndef SYNSLICERENGINE_OBJECT_NOZZLE_H_
#define SYNSLICERENGINE_OBJECT_NOZZLE_H_

#include <Eigen/Core>

namespace SyNSlicerEngine::Object
{
	//!  This class defines a nozzle
	/*!
		   y/2
		--------
		|     /
	 x	|    /
		|   /
		|  /
		---
		 d/2
	*/
	class Nozzle
	{
	public:
		Nozzle();
		Nozzle(const Nozzle &other);
		Nozzle(double d, double x, double y);
		~Nozzle();

		void setNozzle(double d, double x, double y);
		double getD() const;
		double getX() const;
		double getY() const;

		Nozzle &operator=(const Nozzle &other);

	protected:
		double m_d;
		double m_x;
		double m_y;

	};
}

#endif  // SYNSLICERENGINE_OBJECT_NOZZLE_H_