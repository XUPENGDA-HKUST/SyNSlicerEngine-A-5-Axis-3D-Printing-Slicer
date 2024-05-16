#include "partition_selecting_interactor_style.h"

namespace PMP = CGAL::Polygon_mesh_processing;
using SyNSlicerGUI::PartitionSelectingInteractorStyle;

PartitionSelectingInteractorStyle::PartitionSelectingInteractorStyle(vtkRenderer *p_renderer)
    : m_should_drawer_delete_in_destructer(true)
    , mp_renderer(p_renderer)
    , mp_drawer(new ObjectDrawer(p_renderer))
    , m_current_key(100)
{
    LastPickedActor = nullptr;
    vtkNew<vtkNamedColors> colors;
    m_property_for_selected_partition = vtkSmartPointer<vtkProperty>::New();
    m_property_for_selected_partition->SetColor(colors->GetColor3d("Red").GetData());
    m_property_for_selected_partition->SetDiffuse(1.0);
    m_property_for_selected_partition->SetSpecular(0.0);
}

PartitionSelectingInteractorStyle::~PartitionSelectingInteractorStyle()
{
    if (m_should_drawer_delete_in_destructer == true)
    {
        delete mp_drawer;
    }
}

void PartitionSelectingInteractorStyle::OnLeftButtonDown()
{
    vtkNew<vtkNamedColors> colors;

    int *clickPos = this->GetInteractor()->GetEventPosition();

    // Pick from this location.
    vtkNew<vtkPropPicker> picker;
    picker->Pick(clickPos[0], clickPos[1], 0, mp_renderer);

    this->LastPickedActor = picker->GetActor();

    if (this->LastPickedActor)
    {
        if (&(*mp_partitions)[m_mapping_actor_to_partition_list[this->LastPickedActor]] != mp_model_cannot_be_selected)
        {
            ++m_mapping_actor_to_number_of_times_clicked[this->LastPickedActor];
            if (m_mapping_actor_to_number_of_times_clicked[this->LastPickedActor] % 2 == 0)
            {
                // Go back to its original property
                this->LastPickedActor->GetProperty()->DeepCopy(m_property_list[m_mapping_actor_to_partition_list[this->LastPickedActor]]);
            }
            else
            {
                // Change to color to red
                this->LastPickedActor->GetProperty()->DeepCopy(m_property_for_selected_partition);
            }
        }
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}


void PartitionSelectingInteractorStyle::setPartitions(SO::PartitionCollection<CgalMesh_EPICK> &partitions, SO::Plane base_plane)
{
    m_base_plane = base_plane;
    mp_partitions = &partitions;
    SO::PartitionCollection<CgalMesh_EPICK> &m_partitions = partitions;

    m_mapping_actor_to_partition_list.clear();
    m_mapping_actor_to_number_of_times_clicked.clear();

    for (size_t i = 0; i < m_partitions.numberOfPartitions(); i++)
    {
        std::string name("Partition" + std::to_string(i));
        mp_drawer->drawMesh(m_partitions[i].getEPICKMesh(), name);

        vtkSmartPointer<vtkProperty> temp_vtk_property = vtkSmartPointer<vtkProperty>::New();
        temp_vtk_property->SetColor(
            0.2 + 0.6 / m_partitions.numberOfPartitions() * i, 
            0.2 + 0.6 / m_partitions.numberOfPartitions() * i, 
            0.2 + 0.6 / m_partitions.numberOfPartitions() * i);

        mp_drawer->setProperty(name, temp_vtk_property.Get());
        m_property_list.push_back(temp_vtk_property);

        vtkActor *actor = mp_drawer->getObjectDrawn(name)->getActor();

        m_mapping_actor_to_partition_list.insert({ actor,i });
        m_mapping_actor_to_number_of_times_clicked.insert({ actor,0 });

        if (m_partitions[i].getBasePlane() == base_plane)
        {
            mp_model_cannot_be_selected = &m_partitions[i];
        }
    }
}

bool PartitionSelectingInteractorStyle::confirmSelection(SO::PartitionCollection<CgalMesh_EPICK> &partitions)
{
    int number_of_partition_selected = 0;
    std::vector<SO::Partition<CgalMesh_EPICK> *> partition_selected;
    std::vector<SO::Partition<CgalMesh_EPICK> *> partition_not_selected;

    SO::PartitionCollection<CgalMesh_EPICK> &m_partitions = *mp_partitions;

    for (auto it: m_mapping_actor_to_number_of_times_clicked)
    {
        if (it.second % 2 == 1)
        {
            ++number_of_partition_selected;
            partition_selected.push_back(&m_partitions[m_mapping_actor_to_partition_list[it.first]]);
        }
        else
        {
            partition_not_selected.push_back(&m_partitions[m_mapping_actor_to_partition_list[it.first]]);
        }
    }

    if (number_of_partition_selected == 0)
    {
        printf("Nothing is selected.");
        return false;
    }

    printf("%d partitions selected, combine the remaining partitions.\n", int(partition_selected.size()));

    for (int i = 0; i < m_partitions.numberOfPartitions(); i++)
    {
        std::string name("Partition" + std::to_string(i));
        mp_drawer->removeObjectDrawn(name);
    }

    m_result_partitions.clear();

    for (int i = 0; i < partition_selected.size(); i++)
    {
        partition_selected[i]->makeAsCleanAsPossible();
        partition_selected[i]->fillHoles();
        m_result_partitions.addPartition(*partition_selected[i]);
    }

    int base_partition_index = 0;
    for (int i = 0; i < partition_not_selected.size(); i++)
    {
        partition_not_selected[i]->makeAsCleanAsPossible();
        if (partition_not_selected[i]->getBasePlane() == m_base_plane)
        {
            base_partition_index = i;
        }
    }

    for (int i = 0; i < partition_not_selected.size(); i++)
    {
        if (i != base_partition_index)
        {
            partition_not_selected[base_partition_index]->getUnion(partition_not_selected[i]->getEPICKMesh());
        }
    }

    m_result_partitions.addPartition(*partition_not_selected[base_partition_index]);

    partitions = m_result_partitions;

    std::cout << partitions.numberOfPartitions() << " Mesh after selection!" << std::endl;
    mp_drawer->removeAllObjectsDrawn();
    return true; 
}
