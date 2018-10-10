#ifndef PAINTER_OSTREAM_H
#define PAINTER_OSTREAM_H

#include <QPainter>
#include <QPen>
#include <QRectF>
#include <QPainterPath>
#include "Boost_converter.h"
#include "Boost_types.h"




//template <typename K>
class PainterOstream {

private:
  QPainter* qp;
  Qt::Converter convert;
  
  
public:
  PainterOstream(QPainter* p, QRectF rect = QRectF())
    : qp(p)
  {}

  PainterOstream& operator<<(const Point_2& p)
  {
    qp->drawPoint(convert(p));
    return *this;
  }
  
  PainterOstream& operator<<(const Segment_2& s)
  {
    qp->drawLine(convert(s.first), convert(s.second));
    return *this;
  }
  
  
  // PainterOstream& operator<<(const Ray_2& r)
  // {
  //   qp->drawLine(convert(r));
  //   return *this;
  // }

  
  // PainterOstream& operator<<(const Line_2& l)
  // {
  //   qp->drawLine(convert(l));
  //   return *this;
  // }


  // PainterOstream& operator<<(const Triangle_2& t)
  // {
  //   qp->drawPolygon(convert(t));
  //   return *this;
  // }

  // PainterOstream& operator<<(const Iso_rectangle_2& r)
  // {
  //   qp->drawRect(convert(r));
  //   return *this;
  // }

  // PainterOstream& operator<<(const Bbox_2& bb)
  // {
  //   qp->drawRect(convert(bb));
  //   return *this;
  // }

  // PainterOstream& operator<<(const Circle_2& c)
  // {
  //   qp->drawEllipse(convert(c.bbox()));
  //   return *this;
  // }


  // PainterOstream& operator<<(const Circular_arc_point_2& p)
  // {
  //   typedef typename K::Point_2   Point_2;
  //   (*this) << Point_2(to_double(p.x()), to_double(p.y()));
  //   return *this;
  // }


  // PainterOstream& operator<<(const Circular_arc_2& arc)
  // {
  //   const typename K::Circle_2 & circ = arc.supporting_circle();
  //   const typename K::Point_2 & center = circ.center();
  //   const typename K::Circular_arc_point_2 & source = arc.source();
  //   const typename K::Circular_arc_point_2 & target = arc.target();

  //   double asource = std::atan2( -to_double(source.y() - center.y()),
		// 		 to_double(source.x() - center.x())); 
  //   double atarget = std::atan2( -to_double(target.y() - center.y()),
		// 		 to_double(target.x() - center.x()));

  //   std::swap(asource, atarget);
  //   double aspan = atarget - asource;

  //   if(aspan < 0.)
  //     aspan += 2 * CGAL_PI;

  //   const double coeff = 180*16/CGAL_PI;
  //   qp->drawArc(convert(circ.bbox()), 
		// (int)(asource * coeff), 
	 //         (int)(aspan * coeff));
  //   return *this;
  // }

  // PainterOstream& operator<<(const Line_arc_2& arc)
  // {
  //   (*this) << Segment_2(Point_2(to_double(arc.source().x()), to_double(arc.source().y())),
		// 	 Point_2(to_double(arc.target().x()), to_double(arc.target().y())));
  //    return *this;
  // }

  // void draw_parabola_segment(const  Point_2& center, const Line_2& line, 
		// 	     const  Point_2& source, const Point_2& target)
  // {
  //   if (CGAL::collinear(source,target,center))
  //     qp->drawLine(convert(source), convert(target));      
  //   else
  //   {
  //     const Point_2 proj_source = line.projection(source);
  //     const Point_2 proj_target = line.projection(target);      
  //     const Point_2 intersection = circumcenter(proj_source,
  //                                               proj_target,
  //                                               center);
  //     // Property: "intersection" is the intersection of the two tangent
  //     // lines in source and target.
  //     QPainterPath path;
  //     path.moveTo(convert(source));
  //     path.quadTo(convert(intersection), convert(target));
  //     qp->drawPath(path);
  //   }
  // }
};



#endif // PAINTER_OSTREAM_H