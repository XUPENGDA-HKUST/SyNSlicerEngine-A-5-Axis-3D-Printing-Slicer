#ifndef SYNSLICERENGINE_OBJECT_NOZZLE_H_
#define SYNSLICERENGINE_OBJECT_NOZZLE_H_

#include <Eigen/Core>

namespace SyNSlicerEngine::Object
{
	//!  This class defines a nozzle
	//
	//		  y/2
	//		--------
	//		|     /
	//    x	|    /
	//		|   /
	//		|  /
	//		---
	//		d/2

	class Nozzle
	{
	public:
		//! Default constructor.
		Nozzle();

		//! Copy constructor.
		Nozzle(const Nozzle &other);

		//! Constructor
		/*!
			\param[in] d Nozzle diameter.
			\param[in] x See diagram above.
			\param[in] y See diagram above.
		*/
		Nozzle(double d, double x, double y);

		//! Destructore.
		~Nozzle();

		//! Setup the nozzle
		/*!
			\param[in] d Nozzle diameter.
			\param[in] x See diagram above.
			\param[in] y See diagram above.
		*/
		void setNozzle(double d, double x, double y);

		//! Get d.
		double getD() const;

		//! Get x.
		double getX() const;

		//! Get y.
		double getY() const;

		//! Copy assignment operator
		Nozzle &operator=(const Nozzle &other);

	protected:
		//! Store d.
		double m_d;

		//! Store x.
		double m_x;

		//! Store y.
		double m_y;

	};
}

#endif  // SYNSLICERENGINE_OBJECT_NOZZLE_H_