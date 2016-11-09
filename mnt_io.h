/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef CGOGN_IO_MNT_IO_H_
#define CGOGN_IO_MNT_IO_H_

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>

#include <cgogn/modeling/tiling/triangular_grid.h>
#include <cgogn/modeling/tiling/square_grid.h>

#include <iomanip>

namespace cgogn
{

namespace io
{

template<typename VEC3>
class MntSurfaceImport : public FileImport
{
public:
    using Self = MntSurfaceImport<VEC3>;
    using Inherit = FileImport;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;

    inline MntSurfaceImport(){}
    CGOGN_NOT_COPYABLE_NOR_MOVABLE(MntSurfaceImport);
    virtual ~MntSurfaceImport() override {}


    uint32 ncols_, nrows_;
    float64 xllcorner_, yllcorner_;
    float64 cellsize_;

    std::vector<std::vector<Scalar>> data_;

public:

	inline std::vector<std::vector<Scalar>>& data()
    {
        return data_;
    }

    inline uint32 nb_columns()
    {
        return ncols_;
    }

    inline uint32 nb_rows()
    {
        return nrows_;
    }

    inline float64 xll_corner()
    {
        return xllcorner_;
    }

    inline float64 yll_corner()
    {
        return yllcorner_;
    }

    inline float64 cell_size()
    {
        return cellsize_;
    }

    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);
        std::string line;
        line.reserve(512);

        std::size_t found;

        //read header
        std::string key_ncols("ncols");
        std::string key_nrows("nrows");
        std::string key_xllcorner("xllcorner");
        std::string key_yllcorner("yllcorner");
        std::string key_cellsize("cellsize");
        std::string key_NODATA_value("NODATA_value");

        std::getline(fp, line);
        found = line.find(key_ncols);
        if(found != std::string::npos)
        {
            line.erase(found, key_ncols.length());
            ncols_ = uint32(std::stoul(line));
        }

        std::getline(fp, line);
        found = line.find(key_nrows);
        if(found != std::string::npos)
        {
            line.erase(found, key_nrows.length());
            nrows_ = uint32(std::stoul(line));
        }

        std::getline(fp, line);
        found = line.find(key_xllcorner);
        if(found != std::string::npos)
        {
            line.erase(found, key_xllcorner.length());
            xllcorner_ = std::stod(line);
        }

        std::getline(fp, line);
        found = line.find(key_yllcorner);
        if(found != std::string::npos)
        {
            line.erase(found, key_yllcorner.length());
            yllcorner_ = std::stod(line);
        }

        std::getline(fp, line);
        found = line.find(key_cellsize);
        if(found != std::string::npos)
        {
            line.erase(found, key_cellsize.length());
            cellsize_ = std::stod(line);
        }

        std::getline(fp, line);
        found = line.find(key_NODATA_value);
        if(found != std::string::npos)
        {
            line.erase(found, key_NODATA_value.length());
        }

        //read data


        while(std::getline(fp,line))
        {
            std::istringstream iss(line);
            std::vector<Scalar> tmp;
            Scalar a;

            while(iss >> a)
                tmp.push_back(a);

            //skip empty lines
            if(tmp.size() != 0)
                data_.push_back(tmp);
        }

        //verify that's a matrix
        if(data_.size() != 0)
        {
            for(uint32 i = 0 ; i < data_.size() ; ++i)
            {
                if(data_[i].size() != data_[0].size())
                    return false;
            }
        }

        return true;
    }

    template <typename Map>
    void create_map(Map& map,
    				typename Map::template VertexAttribute<VEC3>& attribute)
    {
		cgogn::modeling::TriangularGrid<Map> grid(map, nb_columns()-1, nb_rows()-1);

		std::vector<typename Map::Vertex> vertices = grid.vertices();

		float64 pas = cell_size() / float64(1000.); //km

		float64 dx = xll_corner() / float64(1000.);
		float64 dy = yll_corner() / float64(1000.) + float64(75.);


		uint32 nx = nb_columns()-1;
		uint32 ny = nb_rows()-1;

		for(uint32 i = 0; i <= ny; ++i)
		{
			for(uint32 j = 0; j <= nx; ++j)
			{
				attribute[vertices[i*(nx+1)+j]] =
						VEC3(dx+float64(j)*pas, dy-float64(i)*pas, data()[i][j] / float64(1000.));
			}
		}
    }
};

template <typename VEC3, typename MAP2>
inline void import_height_map(MAP2& cmap2, const std::string& filename)
{	
	typename MAP2::template VertexAttribute<VEC3> vertex_position = cmap2.template add_attribute<VEC3, MAP2::Vertex::ORBIT>("position");

	auto si = cgogn::make_unique<cgogn::io::MntSurfaceImport<VEC3>>();

	if(si)
	{
		if(si->import_file(filename))
		{
			si->create_map(cmap2, vertex_position);
		}
	}
}


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MNT_IO_H_
