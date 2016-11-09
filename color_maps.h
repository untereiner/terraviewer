/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009, IGG Team, LSIIT, University of Strasbourg                *
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
* Web site: http://cgogn.unistra.fr/                                  *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/


#ifndef CGOGN_CORE_UTILS_COLOR_MAPS_H_
#define CGOGN_CORE_UTILS_COLOR_MAPS_H_

#include <array>
#include <cgogn/core/utils/numerics.h>

namespace cgogn
{

inline std::array<float32,3> color_map_blue_green_red(float32 x)
{
    if (x < 0.0f)
        return {0.0f, 0.0f, 1.0f} ;

    if (x < 0.5f)
        return {0.0f, 2.0f * x, 1.0f - 2.0f * x};

    if (x < 1.0f)
        return {2.0f * x - 1.0f, 2.0f - 2.0f * x, 0.0f};

    return {1.0f, 0.0f, 0.0f};
}

inline std::array<float32, 3> color_map_BCGYR(float32 x)
{
    if (x < 0.0f)
        return {0.0f, 0.0f, 1.0f} ;

    if (x < 0.25f)
            return {0.0f, 4.0f * x, 1.0f};

    if (x < 0.5f)
            return {0.0f, 1.0 , 2.0f - 4.0f * x};

    if (x < 0.75f)
            return {4.0f * x - 2.0f, 1.0f, 0.0f};

    if (x < 1.0f)
            return {1.0f, 4.0f - 4.0f * x, 0.0f};

    return {1.0f, 0.0f, 0.0f};
}

} // end namespace cgogn

#endif // CGOGN_CORE_UTILS_COLOR_MAPS_H_

