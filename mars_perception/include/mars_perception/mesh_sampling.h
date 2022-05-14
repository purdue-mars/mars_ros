#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <vtkCellArray.h>

#ifdef VTK_CELL_ARRAY_V2
  using vtkCellPtsPtr = vtkIdType const*;
#else
  using vtkCellPtsPtr = vtkIdType*;
#endif

void polygon_mesh_to_pc(pcl::PolygonMesh *mesh_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr);