/* Terrain generator headers */
#include "graph.h"


namespace raptor_terrain
{

GraphEdge::GraphEdge()
{
    m_v1 = -1;
    m_v2 = -1;
    m_name = -1;
    m_error = 0;
    m_concavity = 0;
    m_deleted = false;
#ifdef HACD_PRECOMPUTE_CHULLS
    m_convexHull = 0;
#endif
}

GraphVertex::GraphVertex()
{
    m_convexHull = 0;
    m_name = -1;
    m_cc = -1;
    m_concavity = 0;
    m_surf = 0;
    m_deleted = false;
}

bool GraphVertex::DeleteEdge(long name)
{
    auto edge_iter = std::find(m_edges.begin(), m_edges.end(), name);
    if (edge_iter != m_edges.end())
    {
        m_edges.erase(edge_iter);
        return true;
    }

    return false;
}

Graph::Graph()
{
    m_nV = 0;
    m_nE = 0;
    m_nCCs = 0;
}

Graph::~Graph()
{
}

void Graph::Allocate(size_t nV, size_t nE)
{ 
    m_nV = nV;
    m_edges.reserve(nE);
    m_vertices.resize(nV);
    for(size_t i = 0; i < nV; i++)
    {
        m_vertices[i].m_name = static_cast<long>(i);
    }
}

long Graph::AddVertex()
{
    size_t name = m_vertices.size();
    m_vertices.resize(name+1);
    m_vertices[name].m_name = static_cast<long>(name);
    m_nV++;
    return static_cast<long>(name);
}

long Graph::AddEdge(long v1, long v2)
{
    size_t name = m_edges.size();
    m_edges.push_back(GraphEdge());
    m_edges[name].m_name = static_cast<long>(name);
    m_edges[name].m_v1 = v1;
    m_edges[name].m_v2 = v2;
    m_vertices[v1].AddEdge(static_cast<long>(name));
    m_vertices[v2].AddEdge(static_cast<long>(name));
    m_nE++;
    return static_cast<long>(name);
}

bool Graph::DeleteEdge(long name)
{
    if (name < static_cast<long>(m_edges.size()))
    {
        long v1 = m_edges[name].m_v1;
        long v2 = m_edges[name].m_v2;
        m_edges[name].m_deleted = true;
        m_vertices[v1].DeleteEdge(name);
        m_vertices[v2].DeleteEdge(name);
        m_nE--;
        return true;
    }
    return false;
}
bool Graph::DeleteVertex(long name)
{
    if (name < static_cast<long>(m_vertices.size()))
    {
        m_vertices[name].m_deleted = true;
        m_vertices[name].m_edges.clear();
        m_vertices[name].m_ancestors = std::vector<long>();
        delete m_vertices[name].m_convexHull;
        m_vertices[name].m_distPoints.clear();
        m_vertices[name].m_boudaryEdges.clear();
        m_vertices[name].m_convexHull = 0;
        m_nV--;
        return true;
    }
    return false;
}


bool Graph::EdgeCollapse(long v1, long v2)
{
    const long edgeToDelete = GetEdgeID(v1, v2);
    if (edgeToDelete >= 0) 
    {
        /* Delete edge */
        DeleteEdge(edgeToDelete);

        /* Add v2 to v1 ancestors */
        m_vertices[v1].m_ancestors.push_back(v2);
        m_vertices[v1].m_ancestors.insert(m_vertices[v1].m_ancestors.begin(), m_vertices[v2].m_ancestors.begin(),  m_vertices[v2].m_ancestors.end());
        
        /* Update adjacency list */
        for(size_t i = 0; i < m_vertices[v2].m_edges.size(); ++i) 
        {
            long b;
            const long idEdge = m_vertices[v2].m_edges[i];
            if (m_edges[idEdge].m_v1 == v2)
            {
                b = m_edges[idEdge].m_v2;
            }
            else
            {
                b = m_edges[idEdge].m_v1;
            }
            if (GetEdgeID(v1, b) >= 0)
            {
                m_edges[idEdge].m_deleted = true;
                m_vertices[b].DeleteEdge(idEdge);
                --m_nE;
            }
            else
            {
                m_edges[idEdge].m_v1 = v1;
                m_edges[idEdge].m_v2 = b;
                
                auto edge_iter = std::find(m_vertices[v1].m_edges.begin(), m_vertices[v1].m_edges.end(), idEdge);
                if (edge_iter == m_vertices[v1].m_edges.end())
                {
                    m_vertices[v1].m_edges.push_back(idEdge);
                }
            }
        }
        
        /* Delete v2 */
        DeleteVertex(v2);            
        return true;
    }
    return false;
}


long Graph::GetEdgeID(long v1, long v2) const
{
    if (v1 < static_cast<long>(m_vertices.size()) && !m_vertices[v1].m_deleted)
    {
        for(size_t i = 0; i < m_vertices[v1].m_edges.size(); ++i) 
        {
            const long idEdge =  m_vertices[v1].m_edges[i];
            if ((m_edges[idEdge].m_v1 == v2) || (m_edges[idEdge].m_v2 == v2))
            {
                return m_edges[idEdge].m_name;
            }
        }
    }
    return -1;
}


void Graph::Print() const
{
    std::cout << "-----------------------------" << std::endl;
    std::cout << "vertices (" << m_nV << ")" << std::endl;
    for (size_t v = 0; v < m_vertices.size(); ++v) 
    {
        const GraphVertex & currentVertex = m_vertices[v];
        if (!m_vertices[v].m_deleted)
        {

            std::cout  << currentVertex.m_name      << "\t";
            long idEdge;
            for(size_t ed = 0; ed < currentVertex.m_edges.size(); ++ed) 
            {
                idEdge = currentVertex.m_edges[ed];
                std::cout  << "(" << m_edges[idEdge].m_v1 << "," << m_edges[idEdge].m_v2 << ") ";       
            }
            std::cout << std::endl;
        }            
    }

    std::cout << "vertices (" << m_nE << ")" << std::endl;
    for (size_t e = 0; e < m_edges.size(); ++e) 
    {
        const GraphEdge & currentEdge = m_edges[e];
        if (!m_edges[e].m_deleted)
        {
            std::cout  << currentEdge.m_name      << "\t(" 
                       << m_edges[e].m_v1          << "," 
                       << m_edges[e].m_v2          << ") "<< std::endl;
        }            
    }
}
void Graph::Clear()
{
    m_vertices.clear();
    m_edges.clear();
    m_nV = 0;
    m_nE = 0;
}

long Graph::ExtractCCs()
{
    // all CCs to -1
    for (size_t v = 0; v < m_vertices.size(); ++v) 
    {
        if (!m_vertices[v].m_deleted)
        {
            m_vertices[v].m_cc = -1;
        }
    }
    
    // we get the CCs
    m_nCCs = 0;
    long v2 = -1;
    long idEdge;
    std::vector<long> temp;
    for (size_t v = 0; v < m_vertices.size(); ++v) 
    {
        if (!m_vertices[v].m_deleted && m_vertices[v].m_cc == -1) 
        {
            m_vertices[v].m_cc = static_cast<long>(m_nCCs);
            temp.clear();
            temp.push_back(m_vertices[v].m_name);
            while (temp.size()) 
            {
                long vertex = temp[temp.size()-1];
                temp.pop_back();                                        
                for(size_t ed = 0; ed < m_vertices[vertex].m_edges.size(); ++ed) 
                {
                    idEdge =  m_vertices[vertex].m_edges[ed];
                    if (m_edges[idEdge].m_v1 == vertex) 
                    {
                        v2 = m_edges[idEdge].m_v2;
                    }
                    else 
                    {
                        v2 = m_edges[idEdge].m_v1;
                    }
                    if ( !m_vertices[v2].m_deleted && m_vertices[v2].m_cc == -1) 
                    {
                        m_vertices[v2].m_cc = static_cast<long>(m_nCCs);
                        temp.push_back(v2);
                    }
                }
            }
            m_nCCs++;
        }
    }        
    return static_cast<long>(m_nCCs);
}
}; /* namespace raptor_terrain */
