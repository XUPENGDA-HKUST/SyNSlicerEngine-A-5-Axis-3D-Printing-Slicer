#include "printing_sequence_determinator.h"

using SyNSlicerEngine::Algorithm::PrintingSequenceDeterminator;

PrintingSequenceDeterminator::PrintingSequenceDeterminator(
	SO::PartitionCollection<CgalMesh_EPICK> &partition_list)
	: m_printing_sequence(partition_list)
{

}

PrintingSequenceDeterminator::~PrintingSequenceDeterminator()
{

}

PrintingSequenceDeterminator::PrintingSequenceStatus PrintingSequenceDeterminator::determinePrintingSequence()
{
	spdlog::info("Determining Printing Sequence");
	this->findSweptVolumeOfNozzleForAllPartition();

	if (this->isSweptVolumeIntersectBuildPlate() == true)
	{
		std::cout << "Nozzle collides with build plate during printing!" << std::endl;
		return PrintingSequenceStatus::CollisionOccurAndCannotBeFixed;
	}
	else
	{
		std::pair<int, int> collided_pair;
		PrintingSequenceStatus status = isPrintingSequenceCollisionFree(m_printing_sequence, collided_pair);
		while (status == PrintingSequenceStatus::CollisionOccurAndCanBeFixed) 
		{
			// If collision occur and analyze as can be fixed, fix it by swapping their printing sequence
			std::swap(m_printing_sequence[collided_pair.first], m_printing_sequence[collided_pair.second]);
			std::swap(m_swept_volume_list[collided_pair.first], m_swept_volume_list[collided_pair.second]);
			status = isPrintingSequenceCollisionFree(m_printing_sequence, collided_pair);
		}

		if (status == PrintingSequenceStatus::CollisionOccurAndCannotBeFixed)
		{
			std::cout << "Collision occur during printing and connot be solved by changing printing sequence!" << std::endl;
			return PrintingSequenceStatus::CollisionOccurAndCannotBeFixed;
		}
		else
		{
			// check if the new printing sequnce is self-supporting
			if (!isPrintingSequenceSelfSupported(m_printing_sequence))
			{
				std::cout << "Not self-supporting for the collision-free printing sequence!" << std::endl;
				return PrintingSequenceStatus::CollisionFreeButNotSelfSupportive;
			}
			else
			{
				std::cout << "Printing sequence now is collision free and self-supporting!" << std::endl;
				std::cout << "-------------------End Determining Printing Sequence----------------" << std::endl;
				return PrintingSequenceStatus::CollisionFreeAndSelfSupportive;
			}
		}
	}	
}

void PrintingSequenceDeterminator::findSweptVolumeOfNozzleForAllPartition()
{
	for (size_t i = 0; i < m_printing_sequence.numberOfPartitions(); i++)
	{
		SweptVolumwCalculator swept_volume_calculator(m_printing_sequence[i]);
		m_swept_volume_list.push_back(swept_volume_calculator.getSweptVolume()[0]);
	}
}

bool PrintingSequenceDeterminator::isSweptVolumeIntersectBuildPlate()
{
	SO::Plane plane;
	double distance = 0.0;
	for (size_t i = 0; i < m_swept_volume_list.size(); i++)
	{
		CgalMesh_EPICK cgal_mesh = m_swept_volume_list[i];
		for (CgalMesh_EPICK::Vertex_index vid: cgal_mesh.vertices())
		{
			CgalPoint_EPICK cgal_point = cgal_mesh.point(vid);
			Eigen::Vector3d point(cgal_point.x(), cgal_point.y(), cgal_point.z());

			distance = plane.getDistanceFromPointToPlane(point);
			if (distance < 0)
			{
				return true;
			}
		}
	}
	return false;
}

bool PrintingSequenceDeterminator::isSweptVolumeIntersectParition(CgalMesh_EPICK swept_volume, CgalMesh_EPICK partition)
{
	CgalMesh_EPICK mesh_out;
	CGAL::Polygon_mesh_processing::corefine_and_compute_intersection(swept_volume, partition, mesh_out);
	if (mesh_out.number_of_vertices() > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool PrintingSequenceDeterminator::isPrintingSequenceSelfSupported(SO::PartitionCollection<CgalMesh_EPICK> &printing_sequence)
{
	std::map<int, bool> parition_and_it_printing_status;

	for (int i = 0; i < printing_sequence.numberOfPartitions(); i++)
	{
		for (auto &key : printing_sequence[i].getKeys())
		{
			parition_and_it_printing_status.insert(std::make_pair(key, false));
		}
	}

	for (int i = 0; i < printing_sequence.numberOfPartitions(); i++)
	{
		if (printing_sequence[i].getLocks().size() == 0)
		{
			for (auto &key : printing_sequence[i].getKeys())
			{
				parition_and_it_printing_status[key] = true;
			}
		}
		else
		{
			for (auto &lock : printing_sequence[i].getLocks())
			{
				if (parition_and_it_printing_status[lock] == false)
				{
					return false;
				}
			}
			for (auto &key : printing_sequence[i].getKeys())
			{
				parition_and_it_printing_status[key] = true;
			}
		}
	}
	return true;
}

PrintingSequenceDeterminator::PrintingSequenceStatus PrintingSequenceDeterminator::isPrintingSequenceCollisionFree(SO::PartitionCollection<CgalMesh_EPICK> &printing_sequence, std::pair<int, int> &collided_pair)
{
	for (int i = 1; i < m_swept_volume_list.size(); i++)
	{
		for (int j = i - 1; j < i; j++)
		{
			if (isSweptVolumeIntersectParition(m_swept_volume_list[i], printing_sequence[j].getEPICKMesh()))
			{
				if (isSweptVolumeIntersectParition(m_swept_volume_list[j], printing_sequence[j].getEPICKMesh()))
				{
					return PrintingSequenceStatus::CollisionOccurAndCannotBeFixed;
				}
				else
				{
					collided_pair = std::pair<int, int>(i, j);
					return PrintingSequenceStatus::CollisionOccurAndCanBeFixed;
				}
			}
		}
	}
	return PrintingSequenceStatus::CollisionFree;
}

