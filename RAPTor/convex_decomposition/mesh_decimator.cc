/* Standard headers */
#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>


/* Terrain generator headers */
#include "mesh_decimator.h"

namespace raptor_terrain
{
MeshDecimator::MeshDecimator(void)
{
    m_triangles                 = 0;
    m_points                    = 0;
    m_nPoints                   = 0;
    m_nInitialTriangles         = 0;
    m_nVertices                 = 0;
    m_nTriangles                = 0;
    m_nEdges                    = 0;
    m_trianglesTags             = 0;
    m_ecolManifoldConstraint    = true;
    m_callBack                  = 0;
}

MeshDecimator::~MeshDecimator(void)
{
    ReleaseMemory();
}


void MeshDecimator::ReleaseMemory()
{
    delete [] m_trianglesTags;
    std::vector< MDVertex > emptyVertices(0);
    m_vertices.swap(emptyVertices);
    std::vector<MDEdge> emptyEdges(0);
    m_edges.swap(emptyEdges);
    m_pqueue = std::priority_queue<MDEdgePriorityQueue, std::vector<MDEdgePriorityQueue>, std::less<MDEdgePriorityQueue> >();
    m_triangles         = 0;
    m_points            = 0;
    m_nPoints           = 0;
    m_nInitialTriangles = 0;
    m_nVertices         = 0;
    m_nTriangles        = 0;
    m_nEdges            = 0;
    m_trianglesTags     = 0;
}


void MeshDecimator::Initialize(size_t nVertices, size_t nTriangles,   Vec3<Float> * points,  Vec3<long> * triangles)
{
    m_nVertices            = nVertices;
    m_nTriangles        = nTriangles;
    m_nInitialTriangles = nTriangles;
    m_points            = points;
    m_nPoints            = nVertices;
    m_triangles            = triangles;
    m_trianglesTags        = new bool[m_nTriangles];
    m_edges.reserve(3*m_nTriangles);
    m_vertices.resize(m_nVertices);
    for(size_t v = 0; v < m_nVertices; ++v)
    {
        m_vertices[v].m_tag = true;
    }
    long tri[3];
    MDEdge edge;
    edge.m_tag = true;
    edge.m_onBoundary = true;
    long nEdges = 0;
    long idEdge;
    long nTris = static_cast<long>(m_nTriangles);
    for(long t = 0; t < nTris; ++t)
    {
        tri[0] = m_triangles[t].X();
        tri[1] = m_triangles[t].Y();
        tri[2] = m_triangles[t].Z();
        m_trianglesTags[t] = true;
        for(int k = 0; k < 3; ++k)
        {
            edge.m_v1 = tri[k];
            edge.m_v2 = tri[(k+1)%3];
            const auto tri_iter = std::find(m_vertices[edge.m_v1].m_triangles.begin(), m_vertices[edge.m_v1].m_triangles.end(), t);
            if (tri_iter == m_vertices[edge.m_v1].m_triangles.end())
            {
                m_vertices[edge.m_v1].m_triangles.push_back(t);
            }

            idEdge = GetEdge(edge.m_v1, edge.m_v2);
            if (idEdge == -1)
            {
                m_edges.push_back(edge);

                const auto edge1_iter = std::find(m_vertices[edge.m_v1].m_edges.begin(), m_vertices[edge.m_v1].m_edges.end(), nEdges);
                if (edge1_iter == m_vertices[edge.m_v1].m_edges.end())
                {
                    m_vertices[edge.m_v1].m_edges.push_back(nEdges);
                }

                const auto edge2_iter = std::find(m_vertices[edge.m_v2].m_edges.begin(), m_vertices[edge.m_v2].m_edges.end(), nEdges);
                if (edge2_iter == m_vertices[edge.m_v2].m_edges.end())
                {
                    m_vertices[edge.m_v2].m_edges.push_back(nEdges);
                }

                ++nEdges;
            }
            else
            {
                m_edges[idEdge].m_onBoundary = false;
            }
        }
    }

    m_nEdges = static_cast<size_t>(nEdges);
    for (size_t v = 0; v < m_nVertices; ++v)
    {
        m_vertices[v].m_onBoundary = false;
        for (size_t itE = 0; itE < m_vertices[v].m_edges.size(); ++itE)
        {
            idEdge = m_vertices[v].m_edges[itE];
            if (m_edges[idEdge].m_onBoundary)
            {
                m_vertices[v].m_onBoundary = true;
                break;
            }
        }
    }
}


long MeshDecimator::GetTriangle(long v1, long v2, long v3) const
{
    for (size_t it = 0; it < m_vertices[v1].m_triangles.size(); ++it)
    {
        long idTriangle = m_vertices[v1].m_triangles[it];
        long i = m_triangles[idTriangle].X();
        long j = m_triangles[idTriangle].Y();
        long k = m_triangles[idTriangle].Z();
        if (((i == v1) && (j == v2) && (k == v3)) || ((i == v1) && (j == v3) && (k == v2)) ||
            ((i == v2) && (j == v1) && (k == v3)) || ((i == v2) && (j == v3) && (k == v1)) ||
            ((i == v3) && (j == v2) && (k == v1)) || ((i == v3) && (j == v1) && (k == v2)))
        {
            return idTriangle;
        }
    }

    return -1;
}
long MeshDecimator::GetEdge(long v1, long v2) const
{
    long idEdge;
    for(size_t it = 0; it < m_vertices[v1].m_edges.size(); ++it)
    {
        idEdge = m_vertices[v1].m_edges[it];
        if ( (m_edges[idEdge].m_v1==v1 && m_edges[idEdge].m_v2==v2) || 
             (m_edges[idEdge].m_v1==v2 && m_edges[idEdge].m_v2==v1) )
        {
            return idEdge;
        }
    }
    return -1;
}
void MeshDecimator::EdgeCollapse(long v1, long v2)
{
    long u, w;
    int shift;
    long idTriangle;
    for(size_t itT = 0; itT < m_vertices[v2].m_triangles.size(); ++itT)
    {
        idTriangle =  m_vertices[v2].m_triangles[itT];
        if (m_triangles[idTriangle].X() == v2)
        {
            shift = 0;
            u = m_triangles[idTriangle].Y();
            w = m_triangles[idTriangle].Z();
        }
        else if (m_triangles[idTriangle].Y() == v2)
        {
            shift = 1;
            u = m_triangles[idTriangle].X();
            w = m_triangles[idTriangle].Z();
        }
        else
        {
            shift = 2;
            u = m_triangles[idTriangle].X();
            w = m_triangles[idTriangle].Y();
        }

        if ((u != v1) && (w != v1) && (GetTriangle(v1, u, w) == -1))
        {
            const auto tri_iter = std::find(m_vertices[v1].m_triangles.begin(), m_vertices[v1].m_triangles.end(), idTriangle);
            if (tri_iter == m_vertices[v1].m_triangles.end())
            {
                m_vertices[v1].m_triangles.push_back(idTriangle);
            }
            m_triangles[idTriangle][shift] = v1;
        }
        else
        {
            m_trianglesTags[idTriangle] = false;
            m_vertices[u].m_triangles.erase(std::find(m_vertices[u].m_triangles.begin(), m_vertices[u].m_triangles.end(), idTriangle));
            m_vertices[w].m_triangles.erase(std::find(m_vertices[w].m_triangles.begin(), m_vertices[w].m_triangles.end(), idTriangle));
            --m_nTriangles;
        }
    }

    for (size_t itE = 0; itE < m_vertices[v2].m_edges.size(); ++itE)
    {
        long idEdge = m_vertices[v2].m_edges[itE];
        w = (m_edges[idEdge].m_v1 == v2)? m_edges[idEdge].m_v2 : m_edges[idEdge].m_v1;
        if ((w != v1) && (GetEdge(v1, w) == -1))
        {
            if (m_edges[idEdge].m_v1 == v2)
            {
                m_edges[idEdge].m_v1 = v1;
            }
            else
            {
                m_edges[idEdge].m_v2 = v1;
            }
    
            const auto edge_iter = std::find(m_vertices[v1].m_edges.begin(), m_vertices[v1].m_edges.end(), idEdge);
            if (edge_iter == m_vertices[v1].m_edges.end())
            {
                m_vertices[v1].m_edges.push_back(idEdge);
            }
        }
        else
        {
            m_edges[idEdge].m_tag = false;
            m_vertices[w].m_edges.erase(std::find(m_vertices[w].m_edges.begin(), m_vertices[w].m_edges.end(), idEdge));
            --m_nEdges;
        }
    }
    m_vertices[v2].m_tag = false;
    m_nVertices--;

    /* Update which edges and vertices are on the boundary */
    for (size_t itE = 0; itE < m_vertices[v1].m_edges.size(); ++itE)
    {
        long idEdge = m_vertices[v1].m_edges[itE];
        const bool on_boundary = (IsBoundaryEdge(m_edges[idEdge].m_v1, m_edges[idEdge].m_v2) != -1);
        long idVertex = (m_edges[idEdge].m_v1 != v1) ? m_edges[idEdge].m_v1 : m_edges[idEdge].m_v2;
        if (on_boundary)
        {
            m_vertices[idVertex].m_onBoundary = true;
        }
        else if (m_edges[idEdge].m_onBoundary && !on_boundary)
        {
            m_vertices[idVertex].m_onBoundary = false;
            for(size_t j = 0; j < m_vertices[idVertex].m_edges.size(); ++j)
            {
                long e = m_vertices[idVertex].m_edges[j];
                if (m_edges[e].m_onBoundary)
                {
                    m_vertices[idVertex].m_onBoundary = true;
                    break;
                }
            }
        }

        m_edges[idEdge].m_onBoundary = on_boundary;
    }

    m_vertices[v1].m_onBoundary = false;
    for(size_t itE = 0; itE < m_vertices[v1].m_edges.size(); ++itE)
    {
        long e = m_vertices[v1].m_edges[itE];
        if (m_edges[e].m_onBoundary)
        {
            m_vertices[v1].m_onBoundary = true;
            break;
        }
    }    
}

long MeshDecimator::IsBoundaryEdge(long v1, long v2) const
{
    long commonTri = -1;
    long itTriangle1, itTriangle2;
    for(size_t itT1 = 0; itT1 < m_vertices[v1].m_triangles.size(); ++itT1)
    {
        itTriangle1 = m_vertices[v1].m_triangles[itT1];
        for(size_t itT2 = 0; itT2 < m_vertices[v2].m_triangles.size(); ++itT2)
        {
            itTriangle2 = m_vertices[v2].m_triangles[itT2];
            if (itTriangle1 == itTriangle2)
            {
                if (commonTri == -1)
                {
                    commonTri = itTriangle1;
                }
                else
                {
                    return -1;
                }
            }
        }
    }
    return commonTri;
}
bool MeshDecimator::IsBoundaryVertex(long v) const
{
    long idEdge;
    for(size_t itE = 0; itE < m_vertices[v].m_edges.size(); ++itE)
    {
        idEdge =  m_vertices[v].m_edges[itE];
        if ( IsBoundaryEdge(m_edges[idEdge].m_v1, m_edges[idEdge].m_v2) != -1) return true;
    }
    return false;
}

void MeshDecimator::GetMeshData(Vec3<Float> * points, Vec3<long> * triangles) const
{
    long * map = new long [m_nPoints];
    long counter = 0;
    for (size_t v = 0; v < m_nPoints; ++v)
    {
        if ( m_vertices[v].m_tag )
        {
            points[counter] = m_points[v];
            map[v] = counter++;
        }
    }
    counter = 0;
    for (size_t t = 0; t < m_nInitialTriangles; ++t)
    {
        if ( m_trianglesTags[t] )
        {
            triangles[counter].X() = map[m_triangles[t].X()];
            triangles[counter].Y() = map[m_triangles[t].Y()];
            triangles[counter].Z() = map[m_triangles[t].Z()];
            counter++;
        }
    }
    delete [] map;
}

void MeshDecimator::InitializeQEM()
{
    Vec3<Float> coordMin = m_points[0];
    Vec3<Float> coordMax = m_points[0];
    Vec3<Float> coord;
    for (size_t p = 1; p < m_nPoints ; ++p) 
    {
        coord = m_points[p];
        if (coordMin.X() > coord.X()) coordMin.X() = coord.X();
        if (coordMin.Y() > coord.Y()) coordMin.Y() = coord.Y();
        if (coordMin.Z() > coord.Z()) coordMin.Z() = coord.Z();
        if (coordMax.X() < coord.X()) coordMax.X() = coord.X();
        if (coordMax.Y() < coord.Y()) coordMax.Y() = coord.Y();
        if (coordMax.Z() < coord.Z()) coordMax.Z() = coord.Z();
    }
    coordMax -= coordMin;
    m_diagBB = coordMax.GetNorm();

    long i, j, k;
    Vec3<Float> n;    
    Float d = 0;
    Float area = 0;
    for(size_t v = 0; v < m_nPoints; ++v)
    {
        memset(m_vertices[v].m_Q, 0, 10 * sizeof(Float));
        long idTriangle;
        for(size_t itT = 0; itT < m_vertices[v].m_triangles.size(); ++itT)
        {
            idTriangle = m_vertices[v].m_triangles[itT];
            i = m_triangles[idTriangle].X();
            j = m_triangles[idTriangle].Y();
            k = m_triangles[idTriangle].Z();
            n = (m_points[j] - m_points[i])^(m_points[k] - m_points[i]);
            area = n.GetNorm();
            n.Normalize();
            d = - (m_points[v] * n);
            m_vertices[v].m_Q[0] += area * (n.X() * n.X());
            m_vertices[v].m_Q[1] += area * (n.X() * n.Y());
            m_vertices[v].m_Q[2] += area * (n.X() * n.Z());
            m_vertices[v].m_Q[3] += area * (n.X() * d);
            m_vertices[v].m_Q[4] += area * (n.Y() * n.Y());
            m_vertices[v].m_Q[5] += area * (n.Y() * n.Z());
            m_vertices[v].m_Q[6] += area * (n.Y() * d);
            m_vertices[v].m_Q[7] += area * (n.Z() * n.Z());
            m_vertices[v].m_Q[8] += area * (n.Z() * d);
            m_vertices[v].m_Q[9] += area * (d     * d);
        }
    }
    Vec3<Float> u1, u2;
    const Float w = static_cast<Float>(1000);
    long t, v1, v2, v3;
    for(size_t e = 0; e < m_edges.size(); ++e)
    {
        v1 = m_edges[e].m_v1;
        v2 = m_edges[e].m_v2;
        t = IsBoundaryEdge(v1, v2);
        if (t != -1)
        {
            if      (m_triangles[t].X() != v1 && m_triangles[t].X() != v2) v3 = m_triangles[t].X();
            else if (m_triangles[t].Y() != v1 && m_triangles[t].Y() != v2) v3 = m_triangles[t].Y();
            else                                                           v3 = m_triangles[t].Z();
            u1 = m_points[v2] - m_points[v1];
            u2 = m_points[v3] - m_points[v1];
            area = w * (u1^u2).GetNorm();
            u1.Normalize();
            n =  u2 - (u2 * u1) * u1;
            n.Normalize();

            d = - (m_points[v1] * n);
            m_vertices[v1].m_Q[0] += area * (n.X() * n.X());
            m_vertices[v1].m_Q[1] += area * (n.X() * n.Y());
            m_vertices[v1].m_Q[2] += area * (n.X() * n.Z());
            m_vertices[v1].m_Q[3] += area * (n.X() * d);
            m_vertices[v1].m_Q[4] += area * (n.Y() * n.Y());
            m_vertices[v1].m_Q[5] += area * (n.Y() * n.Z());
            m_vertices[v1].m_Q[6] += area * (n.Y() * d);
            m_vertices[v1].m_Q[7] += area * (n.Z() * n.Z());
            m_vertices[v1].m_Q[8] += area * (n.Z() * d);
            m_vertices[v1].m_Q[9] += area * (d * d);

            d = - (m_points[v2] * n);
            m_vertices[v2].m_Q[0] += area * (n.X() * n.X());
            m_vertices[v2].m_Q[1] += area * (n.X() * n.Y());
            m_vertices[v2].m_Q[2] += area * (n.X() * n.Z());
            m_vertices[v2].m_Q[3] += area * (n.X() * d);
            m_vertices[v2].m_Q[4] += area * (n.Y() * n.Y());
            m_vertices[v2].m_Q[5] += area * (n.Y() * n.Z());
            m_vertices[v2].m_Q[6] += area * (n.Y() * d);
            m_vertices[v2].m_Q[7] += area * (n.Z() * n.Z());
            m_vertices[v2].m_Q[8] += area * (n.Z() * d);
            m_vertices[v2].m_Q[9] += area * (d * d);
        }
    }
}
void MeshDecimator::InitializePriorityQueue()
{    
    double progressOld = -1.0;
    double progress = 0.0;    
    char msg[1024];
    double ptgStep = 1.0;
    long v1, v2;
    MDEdgePriorityQueue pqEdge;
    size_t nE = m_edges.size();
    for(size_t e = 0; e < nE; ++e)
    {
        progress = e * 100.0 / nE;
        if (fabs(progress-progressOld) > ptgStep && m_callBack)
        {
            sprintf(msg, "%3.2f %% \t \t \r", progress);
            (*m_callBack)(msg, progress, 0.0, m_nVertices);
            progressOld = progress;
        }

        if (m_edges[e].m_tag)
        {
            v1 = m_edges[e].m_v1;
            v2 = m_edges[e].m_v2;
            if ( (!m_ecolManifoldConstraint) || (ManifoldConstraint(v1, v2)))
            {
                pqEdge.m_qem = m_edges[e].m_qem = ComputeEdgeCost(v1, v2, m_edges[e].m_pos);
                pqEdge.m_name = static_cast<long>(e);
                m_pqueue.push(pqEdge);
            }
        }
    }
}


bool MeshDecimator::flips_triangle(const Vec3<Float> &newPos, const Vec3<Float> &oldPosV1, const Vec3<Float> &oldPosV2, const long idTriangle, const long v1, const long v2) const
{
    long a[3];
    a[0] = m_triangles[idTriangle].X();
    a[1] = m_triangles[idTriangle].Y();
    a[2] = m_triangles[idTriangle].Z();

    Vec3<Float> d1_1 = m_points[a[1]] - m_points[a[0]];
    Vec3<Float> d1_2 = m_points[a[2]] - m_points[a[0]];
    Vec3<Float> n1 = d1_1 ^ d1_2;

    m_points[v1] = newPos;
    m_points[v2] = newPos;

    Vec3<Float> d2_1 = m_points[a[1]] - m_points[a[0]];
    Vec3<Float> d2_2 = m_points[a[2]] - m_points[a[0]];
    Vec3<Float> n2 = d2_1 ^ d2_2;

    m_points[v1] = oldPosV1;
    m_points[v2] = oldPosV2;

    n1.Normalize();
    n2.Normalize();
    if ((n1 * n2) < 0.0) 
    {
        return true;
    }

    return false;
}


double MeshDecimator::ComputeEdgeCost(long v1, long v2, Vec3<Float> & newPos) const
{
    double Q[10];
    double M[12];
    Vec3<double> pos;
    for(int i = 0; i < 10; ++i) Q[i] = m_vertices[v1].m_Q[i] + m_vertices[v2].m_Q[i];
    M[0] = Q[0]; // (0, 0)
    M[1] = Q[1]; // (0, 1) 
    M[2] = Q[2]; // (0, 2)
    M[3] = Q[3]; // (0, 3)
    M[4] = Q[1]; // (1, 0)
    M[5] = Q[4]; // (1, 1)
    M[6] = Q[5]; // (1, 2)
    M[7] = Q[6]; // (1, 3)
    M[8] = Q[2]; // (2, 0)
    M[9] = Q[5]; // (2, 1)
    M[10] = Q[7]; // (2, 2);
    M[11] = Q[8]; // (2, 3);
    double det =   M[0] * M[5] * M[10] + M[1] * M[6] * M[8] + M[2] * M[4] * M[9]   
                 - M[0] * M[6] * M[9]  - M[1] * M[4] * M[10]- M[2] * M[5] * M[8]; 
    if (det != 0.0)
    {
        double d = 1.0 / det;
        pos.X() = d * (M[1]*M[7]*M[10] + M[2]*M[5]*M[11] + M[3]*M[6]*M[9]
                      -M[1]*M[6]*M[11] - M[2]*M[7]*M[9]  - M[3]*M[5]*M[10]);
        pos.Y() = d * (M[0]*M[6]*M[11] + M[2]*M[7]*M[8]  + M[3]*M[4]*M[10]
                      -M[0]*M[7]*M[10] - M[2]*M[4]*M[11] - M[3]*M[6]*M[8]);
        pos.Z() = d * (M[0]*M[7]*M[9]  + M[1]*M[4]*M[11] + M[3]*M[5]*M[8]
                      -M[0]*M[5]*M[11] - M[1]*M[7]*M[8]  - M[3]*M[4]*M[9]);
          newPos.X() = static_cast<Float>(pos.X());
        newPos.Y() = static_cast<Float>(pos.Y());
        newPos.Z() = static_cast<Float>(pos.Z());
    }
    else
    {
        const Float w = static_cast<Float>(0.5f);
        newPos = w * m_points[v1] + w * m_points[v2];
          pos.X() = static_cast<double>(newPos.X());
        pos.Y() = static_cast<double>(newPos.Y());
        pos.Z() = static_cast<double>(newPos.Z());
    }

    double qem = pos.X()  * (Q[0] * pos.X() + Q[1] * pos.Y() + Q[2] * pos.Z() + Q[3]) +  
                 pos.Y()  * (Q[1] * pos.X() + Q[4] * pos.Y() + Q[5] * pos.Z() + Q[6]) +
                 pos.Z()  * (Q[2] * pos.X() + Q[5] * pos.Y() + Q[7] * pos.Z() + Q[8]) +
                            (Q[3] * pos.X() + Q[6] * pos.Y() + Q[8] * pos.Z() + Q[9]) ;

    Vec3<Float> oldPosV1 =  m_points[v1];
    Vec3<Float> oldPosV2 =  m_points[v2];
    for (size_t i = 0; i < m_vertices[v1].m_triangles.size(); ++i)
    {
        if (flips_triangle(newPos, oldPosV1, oldPosV2, m_vertices[v1].m_triangles[i], v1, v2))
        {
            return std::numeric_limits<double>::max();
        }
    }

    for (size_t i = 0; i < m_vertices[v2].m_triangles.size(); ++i)
    {
        const long tri = m_vertices[v2].m_triangles[i];
        const auto done_iter = std::find(m_vertices[v1].m_triangles.begin(), m_vertices[v1].m_triangles.end(), tri);
        if ((done_iter == m_vertices[v1].m_triangles.end()) && flips_triangle(newPos, oldPosV1, oldPosV2, tri, v1, v2))
        {
            return std::numeric_limits<double>::max();
        }
    }

    if (m_ecolManifoldConstraint && !ManifoldConstraint(v1, v2))
    {
        return std::numeric_limits<double>::max();
    }

    return qem;
}


bool MeshDecimator::ManifoldConstraint(long v1, long v2) const
{
    long idEdgeV1V2;
    long vertices[4];
    int nr_vertices = 0;
    for (size_t i = 0; i < m_vertices[v1].m_edges.size(); ++i)
    {
        long idEdge1 = m_vertices[v1].m_edges[i];
        long a = (m_edges[idEdge1].m_v1 == v1) ? m_edges[idEdge1].m_v2 : m_edges[idEdge1].m_v1;

        if (nr_vertices < 5)
        {
            auto vert_iter = std::find(vertices, vertices + nr_vertices, a);
            if (vert_iter == (vertices + nr_vertices))
            {
                vertices[nr_vertices++] = a;
            }
        }

        if (a != v2)
        {
            for (size_t j = 0; j < m_vertices[v2].m_edges.size(); ++j)
            {
                long idEdge2 = m_vertices[v2].m_edges[j];
                long b = (m_edges[idEdge2].m_v1 == v2) ? m_edges[idEdge2].m_v2 : m_edges[idEdge2].m_v1;

                if (a == b)
                {
                    if (GetTriangle(v1, v2, a) == -1)
                    {
                        return false;
                    }
                } 
                else if (nr_vertices < 5)
                {
                    auto vert_iter = std::find(vertices, vertices + nr_vertices, b);
                    if (vert_iter == (vertices + nr_vertices))
                    {
                        vertices[nr_vertices++] = b;
                    }
                }
            }
        }
        else
        {
            idEdgeV1V2 = idEdge1;
        }
    }

    if ((nr_vertices < 5) || ( m_vertices[v1].m_onBoundary && m_vertices[v2].m_onBoundary && !m_edges[idEdgeV1V2].m_onBoundary))
    {
        return false;
    }

    return true;
}


bool MeshDecimator::EdgeCollapse(double & qem)
{
    MDEdgePriorityQueue currentEdge;
    long v1, v2;
    bool done = false;
    do
    {
        done = false;
        if (m_pqueue.size() == 0)
        {
            done = true;
            break;
        }
        else
        {
            currentEdge = m_pqueue.top();
            m_pqueue.pop();
        }
    }
    while ( (!m_edges[currentEdge.m_name].m_tag) || (m_edges[currentEdge.m_name].m_qem != currentEdge.m_qem));
    
    if (done) return false;
    v1 = m_edges[currentEdge.m_name].m_v1;
    v2 = m_edges[currentEdge.m_name].m_v2;

    qem = currentEdge.m_qem;
    EdgeCollapse(v1, v2);
    m_points[v1] = m_edges[currentEdge.m_name].m_pos ;
    for (int i = 0; i < 10; ++i)
    {
        m_vertices[v1].m_Q[i] += m_vertices[v2].m_Q[i];
    }

    /* Update edges awaiting processing */
    for(size_t i = 0; i < m_vertices[v1].m_edges.size(); ++i)
    {
        /* Edges from v1 */
        const long e = m_vertices[v1].m_edges[i];
        const long a = m_edges[e].m_v1;
        const long b = m_edges[e].m_v2;

        MDEdgePriorityQueue pqEdge;
        pqEdge.m_qem = m_edges[e].m_qem = ComputeEdgeCost(a, b, m_edges[e].m_pos);
        pqEdge.m_name = e;
        m_pqueue.push(pqEdge);

        /* Edges one away from v1 */
        const long v = (a != v1) ? a : b;
        for(size_t j = 0; j < m_vertices[v].m_edges.size(); ++j)
        {
            const long e = m_vertices[v].m_edges[j];
            const long a = m_edges[e].m_v1;
            const long b = m_edges[e].m_v2;
            if ((a != v1) && (b != v1))
            {
                MDEdgePriorityQueue pqEdge;
                pqEdge.m_qem = m_edges[e].m_qem = ComputeEdgeCost(a, b, m_edges[e].m_pos);
                pqEdge.m_name = e;
                m_pqueue.push(pqEdge);
            }
        }
    }

    return true;    
}
bool MeshDecimator::Decimate(size_t targetNVertices, size_t targetNTriangles, double targetError)
{
    double qem = 0.0;
    double progressOld = -1.0;
    double progress = 0.0;    
    char msg[1024];
    double ptgStep = 1.0;

    if (m_callBack)
    {
        std::ostringstream msg;
        msg << "+ Mesh" << std::endl;
        msg << "\t # vertices                     \t" << m_nPoints << std::endl;
        msg << "\t # triangles                    \t" << m_nTriangles << std::endl;
        msg << "+ Parameters" << std::endl;
        msg << "\t target # of vertices           \t" << targetNVertices << std::endl;
        msg << "\t target # of triangles          \t" << targetNTriangles << std::endl;
        msg << "\t QEM                              \t" << targetError << std::endl;
        (*m_callBack)(msg.str().c_str(), 0.0, 0.0, m_nPoints);
    }
    
    if (m_callBack) (*m_callBack)("+ Initialize QEM \n", 0.0, 0.0, m_nPoints);
    InitializeQEM();
    if (m_callBack) (*m_callBack)("+ Initialize priority queue \n", 0.0, 0.0, m_nPoints);
    InitializePriorityQueue();
    if (m_callBack) (*m_callBack)("+ Simplification \n", 0.0, 0.0, m_nPoints);
    double invDiag2 = 1.0 / (m_diagBB * m_diagBB);
    while((m_pqueue.size() > 0) && 
          (m_nEdges > 0) && 
          (m_nVertices > targetNVertices) &&
          (m_nTriangles > targetNTriangles) &&
          (qem < targetError))
    {
        progress = 100.0 - m_nVertices * 100.0 / m_nPoints;
        if (fabs(progress-progressOld) > ptgStep && m_callBack)
        {
            sprintf(msg, "%3.2f %% V = %lu \t QEM = %f \t \t \r", progress, static_cast<unsigned long>(m_nVertices), sqrt(qem));
            (*m_callBack)(msg, progress, qem, m_nVertices);
            progressOld = progress;
        }
        if (!EdgeCollapse(qem)) break;
        qem *= invDiag2;
    }
    if (m_callBack)
    {
        std::ostringstream msg;
        msg << "+ Simplification output" << std::endl;
        msg << "\t # vertices                     \t" << m_nVertices << std::endl;
        msg << "\t # triangles                    \t" << m_nTriangles << std::endl;
        msg << "\t QEM                              \t" << qem << std::endl;
        (*m_callBack)(msg.str().c_str(), 100.0, qem, m_nVertices);
    }
    return true;
}
}; /* namespace raptor_terrain */
