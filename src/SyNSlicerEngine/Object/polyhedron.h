#ifndef SYNSLICERENGINE_OBJECT_POLYHEDRON_H_
#define SYNSLICERENGINE_OBJECT_POLYHEDRON_H_

#include <CGAL_CORE_CLASS>
#include "spdlog/spdlog.h"

namespace SyNSlicerEngine::Object {

	//!  This class defines a polyhedron.
	/*!
		Polyhedron is store as a CGALMesh.
	*/
	template <class T> 
	class Polyhedron
	{
	public:
		Polyhedron();
		Polyhedron(const Polyhedron &other);
		Polyhedron(std::string file_path);
		Polyhedron(const T &mesh);
		~Polyhedron();

		T &getMesh() const;

		bool writeMeshToSTL(const std::string &name);

		Polyhedron &operator = (const Polyhedron &other);

	protected:
		T m_mesh;

	private:
		//! Loading mesh from a given file path
		bool loadMesh(std::string file_path);
	};

	template<class T>
	inline SyNSlicerEngine::Object::Polyhedron<T>::Polyhedron()
	{
	}

	template<class T>
	inline SyNSlicerEngine::Object::Polyhedron<T>::Polyhedron(const Polyhedron &other)
	{
		*this = other;
	}

	template<class T>
	inline SyNSlicerEngine::Object::Polyhedron<T>::Polyhedron(std::string file_path)
	{
		this->loadMesh(file_path);
	}

	template<class T>
	inline SyNSlicerEngine::Object::Polyhedron<T>::Polyhedron(const T &mesh)
	{
		this->m_mesh = mesh;
	}

	template<class T>
	inline SyNSlicerEngine::Object::Polyhedron<T>::~Polyhedron()
	{
	}

	template<class T>
	inline T &SyNSlicerEngine::Object::Polyhedron<T>::getMesh() const
	{
		return this->m_mesh;
	}

	template<class T>
	inline bool SyNSlicerEngine::Object::Polyhedron<T>::writeMeshToSTL(const std::string &name)
	{
		if (!CGAL::IO::write_polygon_mesh(name, this->m_mesh))
		{
			spdlog::info("Polyhedron::writeEPECKMeshToSTL: Fail!");
			return false;
		}
		return true;
	}

	template<class T>
	inline Polyhedron<T> &SyNSlicerEngine::Object::Polyhedron<T>::operator=(const Polyhedron &other)
	{
		this->m_mesh = other.m_mesh;
		return *this;
	}

	template<class T>
	inline bool SyNSlicerEngine::Object::Polyhedron<T>::loadMesh(std::string file_path)
	{
		std::size_t found_0 = file_path.find(".stl");
		std::size_t found_1 = file_path.find(".STL");

		if (found_0 != std::string::npos || found_1 != std::string::npos)
		{
			if (CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(file_path, this->m_mesh))
			{
				spdlog::info("Read .stl to m_mesh OK");
				return true;
			}
			return false;
		}
		else
		{
			return false;
		}
	}
}

#endif  // SYNSLICERENGINE_OBJECT_POLYHEDRON_H_