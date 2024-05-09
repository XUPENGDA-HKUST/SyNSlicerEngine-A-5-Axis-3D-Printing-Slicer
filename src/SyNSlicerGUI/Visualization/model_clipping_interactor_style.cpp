#include "model_clipping_interactor_style.h"

using SyNSlicerGUI::ModelClippingInteractorStyle;

ModelClippingInteractorStyle::ModelClippingInteractorStyle(vtkRenderer *p_renderer)
    : mp_renderer(p_renderer)
{
    m_line_actor = vtkSmartPointer<vtkActor2D>::New();
    vtkSmartPointer<vtkPolyDataMapper2D> line_mapper_2d =
        vtkSmartPointer<vtkPolyDataMapper2D>::New();
    m_line_actor->GetProperty()->SetColor(1, 0, 0);
    m_line_actor->SetMapper(line_mapper_2d);

    m_actor_added = false;
    m_is_drawing = false;
}

void ModelClippingInteractorStyle::OnMouseWheelForward()
{
    deleteLine();
    vtkInteractorStyleTrackballCamera::OnMouseWheelForward();   
}

void ModelClippingInteractorStyle::OnMouseWheelBackward()
{
    deleteLine();
    vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
}

void ModelClippingInteractorStyle::OnLeftButtonDown()
{
    deleteLine();
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void ModelClippingInteractorStyle::OnRightButtonDown()
{
    int *eventPos = this->Interactor->GetEventPosition();
    m_line_source = Eigen::Vector3d(eventPos[0], eventPos[1], 0); // Record position when click right button mouse
    m_is_drawing = true;
}

void ModelClippingInteractorStyle::OnRightButtonUp()
{
    drawLine(); // Update the line whenever user more the mouse
    m_is_drawing = false;

    int *eventPos = this->Interactor->GetEventPosition();
    m_line_target = Eigen::Vector3d(eventPos[0], eventPos[1], 0);
}

void ModelClippingInteractorStyle::OnMouseMove()
{
    vtkInteractorStyleTrackballCamera::OnMouseMove();
    drawLine();
}

void ModelClippingInteractorStyle::setRenderer(vtkRenderer *p_renderer)
{
    mp_renderer = p_renderer;
    this->SetDefaultRenderer(mp_renderer);
}

void ModelClippingInteractorStyle::setPartition(SO::Partition<CgalMesh_EPICK> *p_partition)
{
    mp_operating_partition = p_partition;
}

SO::Partition<CgalMesh_EPICK> *ModelClippingInteractorStyle::getOperatingPartition()
{
    return mp_operating_partition;
}

void ModelClippingInteractorStyle::drawLine()
{
    if (!m_is_drawing)
    {
        return;
    }

    int *eventPos = this->Interactor->GetEventPosition();
    Eigen::Vector3d pt0 = m_line_source;
    Eigen::Vector3d pt1(eventPos[0], eventPos[1], 0);

    Eigen::Vector3d points[2];
    points[0] = pt0;
    points[1] = pt1;

    vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
    vtk_points->InsertNextPoint(pt0[0], pt0[1], pt0[2]);
    vtk_points->InsertNextPoint(pt1[0], pt1[1], pt1[2]);

    vtkSmartPointer<vtkLine> vtk_line = vtkSmartPointer<vtkLine>::New();
    vtk_line->GetPointIds()->SetId(0, 1);

    vtkSmartPointer<vtkCellArray> vtk_cell_array = vtkSmartPointer<vtkCellArray>::New();
    vtk_cell_array->InsertNextCell(vtk_line);

    vtkSmartPointer<vtkPolyData> vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();
    vtk_poly_data->SetPoints(vtk_points);
    vtk_poly_data->SetLines(vtk_cell_array);
    vtk_poly_data->Modified();
    
    m_line_actor->GetMapper()->SetInputDataObject(vtk_poly_data);
    if (!m_actor_added)
    {
        mp_renderer->AddActor(m_line_actor);
        m_actor_added = true;
    };

    mp_renderer->GetRenderWindow()->Render();
}

bool ModelClippingInteractorStyle::getLine(std::tuple<SO::Line, Eigen::Vector3d, SO::Plane> &line_information)
{
    if (m_actor_added != true)
    {
        printf("Please define the line.");
        return false;
    }

    Eigen::Vector3d pt0 = m_line_source;
    Eigen::Vector3d pt1 = m_line_target;

    mp_renderer->SetDisplayPoint(pt0[0], pt0[1], pt0[2]);
    mp_renderer->DisplayToWorld();
    double pt0_world[4];
    mp_renderer->GetWorldPoint(pt0_world); // From Display Coordinate to World Coordinate
    Eigen::Vector3d pt2(pt0_world);

    mp_renderer->SetDisplayPoint(pt1[0], pt1[1], pt1[2]);
    mp_renderer->DisplayToWorld();
    double pt1_world[4];
    mp_renderer->GetWorldPoint(pt1_world); // From Display Coordinate to World Coordinate
    Eigen::Vector3d pt3(pt1_world);

    Eigen::Vector3d normal;
    mp_renderer->GetActiveCamera()->GetViewPlaneNormal(normal[0], normal[1], normal[2]);

    SO::Line line0(pt2, pt3);

    Eigen::Vector3d camera_pos(mp_renderer->GetActiveCamera()->GetPosition()[0],
        mp_renderer->GetActiveCamera()->GetPosition()[1],
        mp_renderer->GetActiveCamera()->GetPosition()[2]);

    SO::Plane clip_planel(pt2, normal.cross(pt3 - pt2));

    line_information = std::make_tuple(line0, camera_pos, clip_planel);

    return true;
}

void ModelClippingInteractorStyle::deleteLine()
{
    if (m_actor_added == true)
    {
        if (mp_renderer)
        {
            mp_renderer->RemoveActor(m_line_actor);
            mp_renderer->GetRenderWindow()->Render();
            m_actor_added = false;
        }
    };
}

ModelClippingInteractorStyle::~ModelClippingInteractorStyle()
{

}
