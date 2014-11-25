#ifndef __CONVEX_HULL_H__
#define __CONVEX_HULL_H__

/* Standard headers */
#include <vector>
#include <map>

/* Common headers */
#include "point_t.h"

/* Terrain generator headers */
#include "manifold_mesh.h"
#include "vec.h"
#include "allocator.h"

namespace raptor_terrain
{
class DPoint;
class HACD;
//!	Incremental Convex Hull algorithm (cf. http://maven.smith.edu/~orourke/books/ftp.html ).
enum ICHullError
{
	ICHullErrorOK = 0,
	ICHullErrorCoplanarPoints,
	ICHullErrorNoVolume,
	ICHullErrorInconsistent,
	ICHullErrorNotEnoughPoints
};
class ICHull
{
	public:
		static const double									sc_eps;
		//! 
        HeapManager * const     							GetHeapManager() const { return m_heapManager;}
		//!
		void												SetHeapManager(HeapManager * const heapManager)
                                                            { 
                                                                m_heapManager = heapManager;
                                                                m_mesh.SetHeapManager(m_heapManager);
                                                            }
		//!
		bool												IsFlat() { return m_isFlat;}
		//! 
        std::map<long, DPoint> *							GetDistPoints() const { return m_distPoints;}
		//!
		void												SetDistPoints(std::map<long, DPoint> * distPoints) { m_distPoints = distPoints;}
		//! Returns the computed mesh
        TMMesh &                                            GetMesh() { return m_mesh;}
		//!	Add one point to the convex-hull    
        bool                                                AddPoint(const Vec3<Real> & point) {return AddPoints(&point, 1);}
		//!	Add one point to the convex-hull    
        bool                                                AddPoint(const Vec3<Real> & point, long id);
		//!	Add points to the convex-hull
		bool                                                AddPoints(const Vec3<Real> * points, size_t nPoints);	
		bool												AddPoints(std::vector< Vec3<Real> > points);
		//!	
		ICHullError                                         Process();
        //! 
        ICHullError                                         Process(unsigned long nPointsCH);
        //!
        double                                              ComputeVolume();
		//!
		double												ComputeArea();
        //!
        bool                                                IsInside(const Vec3<Real> & pt0, const double eps = 0.0);
		//!
		double												ComputeDistance(long name, const Vec3<Real> & pt, const Vec3<Real> & normal, bool & insideHull, bool updateIncidentPoints);
        //!
        const ICHull &                                      operator=(ICHull & rhs);        

		//!	Constructor
															ICHull(HeapManager * const heapManager=0);
		//! Destructor
		virtual                                             ~ICHull(void) {};

	private:
        //!	DoubleTriangle builds the initial double triangle.  It first finds 3 noncollinear points and makes two faces out of them, in opposite order. It then finds a fourth point that is not coplanar with that face.  The vertices are stored in the face structure in counterclockwise order so that the volume between the face and the point is negative. Lastly, the 3 newfaces to the fourth point are constructed and the data structures are cleaned up. 
		ICHullError                                         DoubleTriangle();
        //!	MakeFace creates a new face structure from three vertices (in ccw order).  It returns a pointer to the face.
        CircularListElement<TMMTriangle> *                  MakeFace(CircularListElement<TMMVertex> * v0,  
                                                                     CircularListElement<TMMVertex> * v1,
                                                                     CircularListElement<TMMVertex> * v2,
                                                                     CircularListElement<TMMTriangle> * fold);			
        //!	
        CircularListElement<TMMTriangle> *                  MakeConeFace(CircularListElement<TMMEdge> * e, CircularListElement<TMMVertex> * v);
		//!	
		bool                                                ProcessPoint();
        //!
        bool                                                ComputePointVolume(double &totalVolume, bool markVisibleFaces);
        //!
        bool                                                FindMaxVolumePoint();
        //!	
        bool                                                CleanEdges();
        //!	
        bool                                                CleanVertices(unsigned long & addedPoints);
        //!	
        bool                                                CleanTriangles();
        //!	
        bool                                                CleanUp(unsigned long & addedPoints);
        //!
        bool                                                MakeCCW(CircularListElement<TMMTriangle> * f,
                                                                    CircularListElement<TMMEdge> * e, 
                                                                    CircularListElement<TMMVertex> * v);
		void												Clear(); 
	private:
        static const long                                   sc_dummyIndex;
        static const double                                 sc_distMin;
		TMMesh                                              m_mesh;
        std::vector<CircularListElement<TMMEdge> *>         m_edgesToDelete;
        std::vector<CircularListElement<TMMEdge> *>         m_edgesToUpdate;
        std::vector<CircularListElement<TMMTriangle> *>     m_trianglesToDelete; 
		std::map<long, DPoint> *							m_distPoints;            
//			CircularListElement<TMMVertex> *					m_dummyVertex;
		Vec3<Real>                                          m_normal;
		bool												m_isFlat;
        HeapManager *                                       m_heapManager;
				                                           
															ICHull(const ICHull & rhs);
    
		friend class HACD;
};
}; /* namespace raptor_terrain */
#endif /* #ifndef __CONVEX_HULL_H__ */
