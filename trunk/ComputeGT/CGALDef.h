#ifndef CGALDEF_H
#define CGALDEF_H

#include <CGAL/Cartesian.h>
#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
#include <CGAL/Circular_kernel_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
// #include <CGAL/Lazy_exact_nt.h>

#include <list>
#include <cmath>
#include <CGAL/number_utils.h>

// #include <CGAL/Cartesian.h>
// #include <CGAL/MP_Float.h>
// #include <CGAL/Lazy_exact_nt.h>
// #include <CGAL/Quotient.h>
// typedef CGAL::Lazy_exact_nt<CGAL::Quotient<CGAL::MP_Float> > NT;
// typedef CGAL::Cartesian<NT> K;

typedef CGAL::Cartesian<float>                                                                  Cartesiano;
//typedef CGAL::Circular_kernel_2<Cartesiano ,  CGAL::Algebraic_kernel_for_circles_2_2 <float> >  K;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Point_2<K> PointCGAL;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Edge_const_iterator EdgeIterator;

typedef CGAL::Polygon_with_holes_2<K>                Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;

#define PHI M_PI *13.0/18.0
#define POINTS_IN_CURVE 10.0

#endif // CGALDEF_H
