#ifndef SYNSLICERGUI_MODELCLIPPINGINTERACTORSTYLE_H_
#define SYNSLICERGUI_MODELCLIPPINGINTERACTORSTYLE_H_

#include <vector>
#include <iostream>

#include "qapplication.h"
#include "qtreewidget.h"
#include <QTreeWidgetItem>
#include <VTK_CORE_CLASS>
#include <Eigen/Core>

#include "Object/partition.h"

namespace SO = SyNSlicerEngine::Object;

//!  A namespace used to store the Class related to Visualization.
namespace SyNSlicerGUI 
{
    //!  This class is the surper class of FinitePlaneClippingInteractorStyle and InfinitePlaneClippingInteractorStyle.
    class ModelClippingInteractorStyle : public vtkInteractorStyleTrackballCamera
    {

    public:
        ModelClippingInteractorStyle(vtkRenderer *p_renderer);
        ~ModelClippingInteractorStyle() override;

        virtual void OnMouseWheelForward() override;
        virtual void OnMouseWheelBackward() override;
        virtual void OnLeftButtonDown() override;
        /*!
          When right clicking the mouse, the starting point of the line is recorded.
        */
        virtual void OnRightButtonDown() override;

        /*!
          When releasing the right clikcing of the mouse, the end point of the line is recorded.
          Clipping is called in this function.
        */
        virtual void OnRightButtonUp() override;
        /*!
          Line is drawn in this function.
        */
        virtual void OnMouseMove() override;

        void setRenderer(vtkRenderer *p_renderer);
        virtual void setPartition(SO::Partition<CgalMesh_EPICK> *p_partition);
        SO::Partition<CgalMesh_EPICK> *getOperatingPartition();

        virtual bool confirmLine() { return false; }

        virtual bool getLine(std::tuple<SO::Line, Eigen::Vector3d, SO::Plane> &line_information);

        void deleteLine();

    protected:
        void drawLine();

        vtkSmartPointer<vtkActor2D> m_line_actor;
        vtkRenderer *mp_renderer;

        Eigen::Vector3d m_line_source;
        Eigen::Vector3d m_line_target;

        bool m_is_drawing;
        bool m_actor_added;

        SO::Partition<CgalMesh_EPICK> *mp_operating_partition;

    private:

    };
}

#endif  // SYNSLICERGUI_MODELCLIPPINGINTERACTORSTYLE_H_