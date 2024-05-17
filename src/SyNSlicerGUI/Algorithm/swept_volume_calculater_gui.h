#ifndef SYNSLICERENGINE_GUI_SWEPTVOLUMECALCULATER_H_
#define SYNSLICERENGINE_GUI_SWEPTVOLUMECALCULATER_H_

#include "SyNSlicerEngine/Algorithm/swept_volume_calculater.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//!  This class is used to calculate volume of the nozzle during printing
	class SweptVolumwCalculatorGUI : public SA::SweptVolumwCalculator
	{
	public:
		//! Default constructor is not allowed.
		SweptVolumwCalculatorGUI() = delete;

		SweptVolumwCalculatorGUI(
			const SO::Partition<CgalMesh_EPICK> &partition, 
			SO::Nozzle nozzle,
			vtkRenderer *p_renderer);
		
		SweptVolumwCalculatorGUI(
			const SO::Partition<CgalMesh_EPICK> &partition,
			SO::Nozzle nozzle,
			SyNSlicerGUI::ObjectDrawer *p_drawer);

		//! Destructor
		~SweptVolumwCalculatorGUI();
		
	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif //SYNSLICERENGINE_GUI_SWEPTVOLUMECALCULATER_H_