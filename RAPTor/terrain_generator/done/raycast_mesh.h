#ifndef __RAYCAST_MESH_H__
#define __RAYCAST_MESH_H__

/* Standard headers */
#include <vector>

/* Terrain generator headers */
#include "vec.h"
#include "safe_array.h"

namespace raptor_terrain
{
typedef double Float;
class RaycastMesh;
class RMNode;
class BBox
{
public:
    bool                                    Raycast(const Vec3<Float> & origin, const Vec3<Float> & dir, Float & distance) const;
                                            BBox(void){}
                                            ~BBox(void){}
private:
    Vec3<Float>                                m_min;    
    Vec3<Float>                                m_max;
    friend class RMNode;
    friend class RaycastMesh;
};

enum RMSplitAxis
{
    RMSplitAxis_X,
    RMSplitAxis_Y,
    RMSplitAxis_Z
};
class RMNode
{
public:
    void                                    ComputeBB();
    bool                                    Raycast(const Vec3<Float> & from, const Vec3<Float> & dir, long & triID, Float & distance, Vec3<Real> & hitPoint, Vec3<Real> & hitNormal) const;
    void                                    Create(size_t depth, size_t maxDepth, size_t minLeafSize, Float minAxisSize);
                                            ~RMNode(void){}
                                            RMNode(void)
                                            {
                                                m_idRight = m_idLeft = m_id = -1;
                                                m_rm = 0;
                                                m_leaf = false;
                                            }
    long                                    m_id;
    long                                    m_idLeft;
    long                                    m_idRight;
    BBox                                    m_bBox;        
    std::vector<long>                       m_triIDs;
    RaycastMesh *                           m_rm;
    bool                                    m_leaf;
};

class RaycastMesh
{
public:
size_t                                        GetNNodes() const { return m_nNodes;}
size_t                                        AddNode() { m_nNodes++; return m_nNodes-1; }
void                                        ComputeBB();
bool                                        Raycast(const Vec3<Float> & from, const Vec3<Float> & dir, long & triID, Float & distance, Vec3<Real> & hitPoint, Vec3<Real> & hitNormal) const;
void                                        Initialize(size_t nVertices, size_t nTriangles, 
                                                       Vec3<Float> *  vertices,  Vec3<long> * triangles, 
                                                       size_t maxDepth=15, size_t minLeafSize = 4, Float minAxisSize = 2.0);    
                                            RaycastMesh(void);
                                            ~RaycastMesh(void);
private : 

private:
    Vec3<long> *                            m_triangles;            
    Vec3<Float> *                            m_vertices;        
    size_t                                    m_nVertices;
    size_t                                    m_nTriangles;
    RMNode *                                m_nodes;
    BBox                                    m_bBox;
    size_t                                    m_nNodes;
    size_t                                    m_nMaxNodes;
    friend class RMNode;
};
}; /* namespace raptor_terrain */
#endif /* #ifndef __RAYCAST_MESH_H__ */
