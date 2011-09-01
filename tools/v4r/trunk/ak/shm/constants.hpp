/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader and Bernhard Miller                *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef SHARED_MEM_CONSTANTS_HPP
#define SHARED_MEM_CONSTANTS_HPP

///Inlcude this file after "ak/sharedmem/sharedmem.hpp"

#define SHARED_MEMORY_TYPE_IMAGE                 (12 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_IMAGEDATA             (14 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_PROCESS_INFORMATION   (15 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_POINT2D64             (16 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_LINES2D64             (17 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_OBJECT                (20 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_STRING                (21 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_JOINTATOMICACTION     (22 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_WALK                  (23 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_SENSORS               (24 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_CAMERASETTINGS        (25 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_CAMERAPARAMETER       (26 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_ACTUATORS             (27 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_VELOCITIES            (28 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_ROBOCUPGAMECONTROLDATA    (29 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_ROBOCUPGAMECONTROLRETURNDATA  (30 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_VEC2D                 (31 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_VEC3D                 (32 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_VEC4D                 (33 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_EXTRINSIC             (34 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_POSITION              (35 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_GAMECONFIG            (36 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_GOALCOLORS            (37 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_GEOMETRY_HELPER       (38 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_WORLDMODELSTRUCT      (39 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_DISTORTION            (40 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_INTRINSIC             (41 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHARED_MEMORY_TYPE_POSE3D64F             (45 + AK::SharedMem::TYPE_CUSTOMIZED)


///V4R
#define SHM_TYPE_V4R									 		(1000 + AK::SharedMem::TYPE_CUSTOMIZED)
#define SHM_TYPE_V4RMARKER                (1 + SHM_TYPE_V4R)
#define SHM_TYPE_V4RCAMGEO              	(3 + SHM_TYPE_V4R)

#endif /* CONSTANTS_HPP */
