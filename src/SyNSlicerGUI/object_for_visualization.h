#ifndef SYNSLICERGUI_OBJECTFORVISUALIZATION_H_
#define SYNSLICERGUI_OBJECTFORVISUALIZATION_H_

#include <iostream>
#include <vector>

#include <VTK_CORE_CLASS>

#include "Object/line.h"
#include "Object/plane.h"

namespace SO = SyNSlicerEngine::Object;

namespace GUI {

	//!  This class is used to store a object for visualizing in the renderer 
	/*!
		For every item you want to display, add this one into item's private member
		Then create the item's polydata, use setPolyData() to store the polydata and then call addToRenderer() to show the item on screen
		Delete this class will automatically remove the item from the screen.
	*/
	class ObjectForVisualization
	{
	public:
		enum InputType { NoInput, PolyData, PolyDataMapper, Actor };

		ObjectForVisualization();
		ObjectForVisualization(const ObjectForVisualization &other);
		~ObjectForVisualization();
		
		//! Call this to initialize class members 
		/*!
			For memory efficiency, class memebers will be constructed only if this function is called.
		*/
		void prepareForVisualization();

		//! InputType = PolyData if this function is called.
		void setPolyData(vtkPolyData *p_polydata);
		//! InputType = PolyDataMapper if this function is called.
		void setPolyDataMapper(vtkPolyDataMapper *p_poly_data_mapper);
		//! InputType = Actor if this function is called.
		void setActor(vtkActor *p_actor);

		void setColor(double r, double g, double b);
		void setOpacity(double opacity);
		void setVisible(bool visibility);
		void setEdgeVisible(bool visibility);

		virtual vtkPolyData *getPolyData();
		virtual vtkActor *getActor();
		virtual vtkProperty *getProperty();

		void addToRenderer(vtkRenderer *p_renderer);
		void removeFromRenderer();

		ObjectForVisualization &operator=(const ObjectForVisualization &other);

	protected:
		virtual void update();

		vtkPolyData *mp_poly_data;
		vtkPolyDataMapper *mp_poly_data_mapper;
		vtkActor *mp_actor;
		vtkProperty *mp_property;

	private:
		bool ready_to_be_visualized;
		vtkRenderer *mp_renderer;
		InputType m_input_type; //!< InputType determine how to perform update();
	};

	class TriangleForVisualization : public ObjectForVisualization
	{
	public:
		TriangleForVisualization();
		TriangleForVisualization(std::vector<Eigen::Vector3d> points);
		~TriangleForVisualization();

		void addToRenderer(vtkRenderer *p_renderer);
	};
}

#endif  // SYNSLICERGUI_OBJECTFORVISUALIZATION_H_