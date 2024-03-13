#include "object_for_visualization.h"

using GUI::ObjectForVisualization;
using GUI::TriangleForVisualization;

ObjectForVisualization::ObjectForVisualization()
	: mp_poly_data(nullptr)
	, mp_poly_data_mapper(nullptr)
	, mp_actor(nullptr), mp_property(nullptr)
	, ready_to_be_visualized(false), mp_renderer(nullptr), m_input_type(NoInput)
{
	std::cout << "ObjectForVisualization()" << std::endl;
}

ObjectForVisualization::ObjectForVisualization(const ObjectForVisualization &other)
	: mp_poly_data(nullptr)
	, mp_poly_data_mapper(nullptr)
	, mp_actor(nullptr), mp_property(nullptr)
	, ready_to_be_visualized(false), mp_renderer(nullptr), m_input_type(NoInput)
{
	*this = other;
}

ObjectForVisualization::~ObjectForVisualization()
{
	if (ready_to_be_visualized == true)
	{
		removeFromRenderer();
		mp_poly_data->Delete();
		mp_poly_data_mapper->Delete();
		mp_actor->Delete();
		ready_to_be_visualized = false;
	}
	std::cout << "~ObjectForVisualization()" << std::endl;
}

void ObjectForVisualization::prepareForVisualization()
{
	if (ready_to_be_visualized == false)
	{
		mp_poly_data = vtkPolyData::New();
		mp_poly_data_mapper = vtkPolyDataMapper::New();
		mp_actor = vtkActor::New();
		update();
		ready_to_be_visualized = true;
	}
}

void ObjectForVisualization::setPolyData(vtkPolyData *p_polydata)
{
	if (p_polydata != nullptr)
	{
		prepareForVisualization();
		mp_poly_data->DeepCopy(p_polydata);
		m_input_type = PolyData;
		update();
	}
}

void ObjectForVisualization::setPolyDataMapper(vtkPolyDataMapper *p_poly_data_mapper)
{
	if (p_poly_data_mapper!=nullptr)
	{
		prepareForVisualization();
		mp_poly_data_mapper->Delete();
		mp_poly_data_mapper = p_poly_data_mapper;
		m_input_type = PolyDataMapper;
		update();
	}
}

void ObjectForVisualization::setActor(vtkActor *p_actor)
{
	if (p_actor != nullptr)
	{
		prepareForVisualization();
		mp_actor->Delete();
		mp_actor = p_actor;
		m_input_type = Actor;
		update();
	}
}

void ObjectForVisualization::setColor(double r, double g, double b)
{
	if (ready_to_be_visualized == true && mp_renderer != nullptr)
	{
		if (r > 1.0)
		{
			r = r / 255;
		}
		if (g > 1.0)
		{
			g = g / 255;
		}
		if (b > 1.0)
		{
			b = b / 255;
		}
		mp_actor->GetProperty()->SetColor(r, g, b);
		mp_renderer->GetRenderWindow()->Render();
	}
}

void ObjectForVisualization::setOpacity(double opacity)
{
	if (ready_to_be_visualized == true && mp_renderer != nullptr)
	{
		if (opacity > 1.0)
		{
			mp_actor->GetProperty()->SetOpacity(1);
		}
		else if (opacity < 0.0)
		{
			mp_actor->GetProperty()->SetOpacity(0);
		}
		else
		{
			mp_actor->GetProperty()->SetOpacity(opacity);
		}
		mp_renderer->GetRenderWindow()->Render();
	}
}

void ObjectForVisualization::setVisible(bool visibility)
{
	if (ready_to_be_visualized && mp_renderer != nullptr)
	{
		mp_actor->SetVisibility(visibility);
		mp_renderer->GetRenderWindow()->Render();
	}
}

void ObjectForVisualization::setEdgeVisible(bool visibility)
{
	if (ready_to_be_visualized && mp_renderer != nullptr)
	{
		mp_property->SetEdgeVisibility(visibility);
		mp_renderer->GetRenderWindow()->Render();
	}
}

vtkPolyData *ObjectForVisualization::getPolyData()
{
	return mp_poly_data;
}

vtkActor *ObjectForVisualization::getActor()
{
	if (ready_to_be_visualized)
	{
		update();
	}
	return mp_actor;
}

vtkProperty *ObjectForVisualization::getProperty()
{
	return mp_property;
}

void ObjectForVisualization::addToRenderer(vtkRenderer *p_renderer)
{
	prepareForVisualization();
	update();
	mp_renderer = p_renderer;
	mp_renderer->AddActor(mp_actor);
	mp_renderer->GetRenderWindow()->Render();
}

void ObjectForVisualization::removeFromRenderer()
{
	if (mp_renderer == nullptr)
	{
		//std::cout << "This object is not yet added to a renderer!" << std::endl;
	}
	else
	{
		mp_renderer->RemoveActor(mp_actor);
		mp_renderer = nullptr;
	}
}

ObjectForVisualization &ObjectForVisualization::operator=(const ObjectForVisualization &other)
{
	if (other.ready_to_be_visualized == false)
	{
		if (ready_to_be_visualized == true)
		{
			removeFromRenderer();
			mp_poly_data->Delete();
			mp_poly_data = nullptr;
			mp_poly_data_mapper->Delete();
			mp_poly_data_mapper = nullptr;
			mp_actor->Delete();
			mp_actor = nullptr;
			mp_renderer = nullptr;
			m_input_type = other.m_input_type;
			ready_to_be_visualized = false;
		}
		else
		{
			mp_poly_data = nullptr;
			mp_poly_data_mapper = nullptr;
			mp_actor = nullptr;
			mp_renderer = nullptr;
			m_input_type = other.m_input_type;
			ready_to_be_visualized = false;
		}
	}
	else
	{
		switch (other.m_input_type)
		{
		case NoInput:
			break;
		case PolyData:
			setPolyData(other.mp_poly_data);
			break;
		case PolyDataMapper:
			setPolyDataMapper(other.mp_poly_data_mapper);
			break;
		case Actor:
			setActor(other.mp_actor);
			break;
		}
	}
	return *this;
}

void ObjectForVisualization::update()
{
	switch (m_input_type)
	{
	case NoInput:
		break;
	case PolyData:
		mp_poly_data_mapper->SetInputData(mp_poly_data);
		vtkPolyDataMapper::SetResolveCoincidentTopologyToPolygonOffset();
		mp_actor->SetMapper(mp_poly_data_mapper);
		mp_property = mp_actor->GetProperty();
		break;
	case PolyDataMapper:
		vtkPolyDataMapper::SetResolveCoincidentTopologyToPolygonOffset();
		mp_actor->SetMapper(mp_poly_data_mapper);
		mp_property = mp_actor->GetProperty();
		break;
	case Actor:
		mp_property = mp_actor->GetProperty();
		break;
	}
}

TriangleForVisualization::TriangleForVisualization()
{
}

TriangleForVisualization::TriangleForVisualization(std::vector<Eigen::Vector3d> vertices)
{
	// Create a triangle.
	vtkNew<vtkPoints> points;
	points->InsertNextPoint(vertices[0][0], vertices[0][1], vertices[0][2]);
	points->InsertNextPoint(vertices[1][0], vertices[1][1], vertices[1][2]);
	points->InsertNextPoint(vertices[2][0], vertices[2][1], vertices[2][2]);

	vtkNew<vtkTriangle> triangle;
	triangle->GetPointIds()->SetId(0, 0);
	triangle->GetPointIds()->SetId(1, 1);
	triangle->GetPointIds()->SetId(2, 2);

	vtkNew<vtkCellArray> triangles;
	triangles->InsertNextCell(triangle);

	// Create a polydata object
	vtkNew<vtkPolyData> trianglePolyData;

	// Add the geometry and topology to the polydata.
	trianglePolyData->SetPoints(points);
	trianglePolyData->SetPolys(triangles);

	this->setPolyData(trianglePolyData);
}

TriangleForVisualization::~TriangleForVisualization()
{
}

void TriangleForVisualization::addToRenderer(vtkRenderer *p_renderer)
{
	ObjectForVisualization::addToRenderer(p_renderer);
}