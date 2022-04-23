//
// LICENCE:
//  The MIT License (MIT)
//
//  Copyright (c) 2016 Karim Naaji, karim.naaji@gmail.com
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE
//
// REFERENCES:
//  http://matthias-mueller-fischer.ch/publications/tetraederCollision.pdf
//  http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox2.txt
//
// HOWTO:
//  #define VOXELIZER_IMPLEMENTATION
//  #define VOXELIZER_DEBUG // Only if assertions need to be checked
//  #include "voxelizer.h"
//
// HISTORY:
//  - 0.10.0 (20-03-2017): Add vx_voxelize_snap_3d_grid to voxelize to 3d-textures
//  - 0.9.2  (03-01-2017): Fix triangle bounding bouxes bounds for bbox-triangle
//                         intersection test
//  - 0.9.1  (12-10-2016): Add vx_voxelize_pc to generate point cloud as a result
//                         of voxelization
//  - 0.9    (01-05-2016): Initial
//
// TODO:
//  - Triangle face merging
//



#ifndef VOXELIZER_H
#define VOXELIZER_H

// ------------------------------------------------------------------------------------------------
// VOXELIZER PUBLIC API
//

#ifndef VOXELIZER_HELPERS
#include <stdlib.h> // malloc, calloc, free
#endif

typedef struct vx_vertex {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        struct {
            float r;
            float g;
            float b;
        };
    };
} vx_vertex_t;

typedef vx_vertex_t vx_vec3_t;
typedef vx_vertex_t vx_color_t;

typedef struct vx_mesh {
    vx_vertex_t* vertices;          // Contiguous mesh vertices
    vx_color_t* colors;             // Contiguous vertices colors
    vx_vec3_t* normals;             // Contiguous mesh normals
    unsigned int* indices;          // Mesh indices
    unsigned int* normalindices;    // Mesh normal indices
    size_t nindices;                // The number of normal indices
    size_t nvertices;               // The number of vertices
    size_t nnormals;                // The number of normals
} vx_mesh_t;

typedef struct vx_point_cloud {
    vx_vertex_t* vertices;          // Contiguous point cloud vertices positions, each vertex corresponds
                                    // to the center of a voxel
    vx_color_t* colors;             // Contiguous point cloud vertices colors
    size_t nvertices;               // The number of vertices in the point cloud
} vx_point_cloud_t;

//
void vx__add_voxel(vx_mesh_t* mesh,
                   vx_vertex_t* pos,
                   vx_color_t color,
                   float* vertices);

// vx_voxelize_pc: Voxelizes a triangle mesh to a point cloud
vx_point_cloud_t* vx_voxelize_pc(vx_mesh_t const* mesh, // The input mesh
                                 float voxelsizex,      // Voxel size on X-axis
                                 float voxelsizey,      // Voxel size on Y-axis
                                 float voxelsizez,      // Voxel size on Z-axis
                                 float precision);      // A precision factor that reduces "holes artifact
                                                        // usually a precision = voxelsize / 10. works ok

// vx_voxelize: Voxelizes a triangle mesh to a triangle mesh representing cubes
vx_mesh_t* vx_voxelize(vx_mesh_t const* mesh,       // The input mesh
        float voxelsizex,                           // Voxel size on X-axis
        float voxelsizey,                           // Voxel size on Y-axis
        float voxelsizez,                           // Voxel size on Z-axis
        float precision);                           // A precision factor that reduces "holes" artifact
                                                    // usually a precision = voxelsize / 10. works ok.

// vx_voxelize_snap_3d_grid: Voxelizes a triangle mesh to a 3d texture
// The texture data is aligned as RGBA8 and can be uploaded as a 3d texture with OpenGL like so:
// glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, width, height, depth, 0, GL_RGBA, GL_UNSIGNED_BYTE, texturedata);
unsigned int* vx_voxelize_snap_3dgrid(vx_mesh_t const* mesh, // The input mesh
        unsigned int width,                                  // The texture resolution on x-axis
        unsigned int height,                                 // The texture resolution on y-axis
        unsigned int depth);                                 // The texture resolution on z-axis


// Allocates a mesh that can contain nvertices vertices, nindices indices
vx_mesh_t* vx_mesh_alloc(int nvertices, int nindices);

// Allocates a mesh that can contain nvertices vertices and colors, nindices indices
vx_mesh_t* vx_color_mesh_alloc(int nvertices, int nindices);

// Free a mesh allocated with vx_mesh_alloc, vx_color_mesh_alloc or after a call to vx_voxelize
void vx_mesh_free(vx_mesh_t* mesh);

// Free a point cloud allocated after a call of vx_voxelize_pc
void vx_point_cloud_free(vx_point_cloud_t* pointcloud);

// Voxelizer Helpers, define your own if needed
#ifndef VOXELIZER_HELPERS
#define VOXELIZER_HELPERS 1
#define VX_MIN(a, b) (a > b ? b : a)
#define VX_MAX(a, b) (a > b ? a : b)
#define VX_FINDMINMAX(x0, x1, x2, min, max) \
    min = max = x0;                         \
    if (x1 < min) min = x1;                 \
    if (x1 > max) max = x1;                 \
    if (x2 < min) min = x2;                 \
    if (x2 > max) max = x2;
#define VX_CLAMP(v, lo, hi) VX_MAX(lo, VX_MIN(hi, v))
#define VX_MALLOC(T, N) ((T*) malloc(N * sizeof(T)))
#define VX_FREE(T) free(T)
#define VX_CALLOC(T, N) ((T*) calloc(N * sizeof(T), 1))
#define VX_SWAP(T, A, B) { T tmp = B; B = A; A = tmp; }
#ifdef VOXELIZER_DEBUG
#define VX_ASSERT(STMT) if (!(STMT)) { *(int *)0 = 0; }
#else
#define VX_ASSERT(STMT)
#endif // VOXELIZER_DEBUG
#endif // VOXELIZER_HELPERS

//
// END VOXELIZER PUBLIC API
// ------------------------------------------------------------------------------------------------

#endif // VOXELIZER_H