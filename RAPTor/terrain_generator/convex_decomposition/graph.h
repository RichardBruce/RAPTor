#ifndef __GRAPH_H__
#define __GRAPH_H__

/* Standard headers */
#include <algorithm>
#include <map>
#include <vector>

/* Terrain generator headers */
#include "vec.h"
#include "convex_hull.h"


//#define HACD_PRECOMPUTE_CHULLS

namespace raptor_terrain
{
class GraphVertex;
class GraphEdge;
class Graph;
class HACD;
   
class GraphVertex  
{
public:
    bool                                                      AddEdge(long name) 
                                                              { 
                                                                auto edge_iter = std::find(m_edges.begin(), m_edges.end(), name);
                                                                if (edge_iter == m_edges.end())
                                                                {
                                                                    m_edges.push_back(name);
                                                                }

                                                                return true; 
                                                              }
    bool                                                      DeleteEdge(long name);        
                                                              GraphVertex();
                                                              ~GraphVertex(){ delete m_convexHull;};      
private:
    long                                                      m_name;
    long                                                      m_cc;
    std::vector<long>                                         m_edges;
    bool                                                      m_deleted;
    std::vector<long>                                         m_ancestors;
    std::vector<DPoint>                                       m_distPoints;

    Real                                                      m_concavity;
    double                                                    m_surf;
    ICHull *                                                  m_convexHull;
    std::vector<unsigned long long>                           m_boudaryEdges;
    

    friend class GraphEdge;
    friend class Graph;
    friend class HACD;
};

class GraphEdge 
{
public:
                                                             GraphEdge();
                                                             ~GraphEdge()
                                                             {
#ifdef HACD_PRECOMPUTE_CHULLS
                                                                delete m_convexHull;
#endif
                                                             };
private:
    long                                                     m_name;
    long                                                     m_v1;
    long                                                     m_v2;
    double                                                   m_concavity;
    Real                                                     m_error;
#ifdef HACD_PRECOMPUTE_CHULLS
    ICHull *                                                 m_convexHull;
#endif
    bool                                                     m_deleted;
    
    friend class GraphVertex;
    friend class Graph;
    friend class HACD;
};

class Graph  
{
public:
    size_t                                                     GetNEdges() const { return m_nE;}
    size_t                                                     GetNVertices() const { return m_nV;}
    bool                                                     EdgeCollapse(long v1, long v2);
    long                                                     AddVertex();
    long                                                     AddEdge(long v1, long v2);
    bool                                                     DeleteEdge(long name);    
    bool                                                     DeleteVertex(long name);
    long                                                     GetEdgeID(long v1, long v2) const;
    void                                                     Clear();
    void                                                     Print() const;
    long                                                     ExtractCCs();
    
                                                             Graph();
    virtual                                                  ~Graph();      
    void                                                     Allocate(size_t nV, size_t nE);

private:
    size_t                                                   m_nCCs;
    size_t                                                   m_nV;
    size_t                                                   m_nE;
    std::vector<GraphEdge>                                   m_edges;
    std::vector<GraphVertex>                                 m_vertices;

    friend class HACD;
};
}; /* namespace raptor_terrain */
#endif /* #ifndef __GRAPH_H__ */
