// Copyright (c) 2008  GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
// 
//
// Author(s)     : Andreas Fabri <Andreas.Fabri@geometryfactory.com>
//                 Laurent Rineau <Laurent.Rineau@geometryfactory.com>
#ifndef CGAL_QT_CONVERTER_H
#define CGAL_QT_CONVERTER_H

#include <QPointF>
#include <QLineF>
#include <QRectF>
#include <QPolygonF>
#include <list>

#include <boost/mpl/has_xxx.hpp>
#include "Boost_types.h"


namespace Qt {

class Converter {

private:
  bool clippingRectIsInitialized;
  //CGAL_Iso_rectangle_2 clippingRect;
  

public:

  Converter()
    : clippingRectIsInitialized(false)
  {
  }

  // Converter(QRectF rect)
  // {
  //   if(rect.isValid()) {
  //     clippingRect = this->operator()(rect);
  //     clippingRectIsInitialized = true;
  //   }
  //   else
  //     clippingRectIsInitialized = false;
  // }

  Point_2 operator()(const QPointF& qp) const
  {
    return Point_2(qp.x(), qp.y());
  }


  QPointF operator()(const Point_2& p) const
  {
    return QPointF(p.get<0>(), p.get<1>() );
  }

  QRectF operator()(const Box_2& b) const
  {
    QPointF topLeft, bottomRight;
    topLeft = QPointF(b.min_corner().get<0>(), b.min_corner().get<1>());
    bottomRight = QPointF(b.max_corner().get<0>(), b.max_corner().get<1>());
    return QRectF(topLeft, bottomRight);
  }
  // QPointF operator()(const CGAL_Circular_arc_point_2& p) const
  // {
  //   return QPointF(to_double(p.x()), to_double(p.y()));
  // }

      
  // CGAL_Segment_2 operator()(const QLineF& qs) const
  // {
  //   return CGAL_Segment_2(operator()(qs.p1()), operator()(qs.p2()));
  // }
 
  // QLineF operator()(const CGAL_Segment_2 &s) const
  // {
  //   return QLineF(operator()(s.source()), operator()(s.target()));
  // }

  
  // CGAL_Iso_rectangle_2 operator()(const QRectF& qr) const
  // {
  //   return CGAL_Iso_rectangle_2(operator()(qr.bottomLeft()), operator()(qr.topRight()));
  // }


  // QRectF operator()(const CGAL_Iso_rectangle_2& r) const
  // {
  //   return QRectF(operator()(r[3]), operator()(r[1])).normalized();  // top left, bottom right
  // }


  // QRectF operator()(const CGAL::Bbox_2& bb) const
  // {
  //   return QRectF(bb.xmin(),
		//   bb.ymin(),
		//   bb.xmax()-bb.xmin(),
		//   bb.ymax()-bb.ymin());
  // }

     
  // QLineF operator()(const CGAL_Ray_2 &r) const
  // {
  //   CGAL_assertion(clippingRectIsInitialized);
  //   Object o = CGAL::intersection(r, clippingRect);
  //   typedef CGAL_Segment_2 Segment_2;
  //   typedef CGAL_Point_2 Point_2;
  //   if(const Segment_2 *s = CGAL::object_cast<Segment_2>(&o)){
  //     return this->operator()(*s);
  //   } else if(const Point_2 *p = CGAL::object_cast<Point_2>(&o)){
  //     return QLineF(operator()(*p), operator()(*p));
  //   }
  //   return QLineF();
  // }

  // QLineF operator()(const CGAL_Line_2 &l) const
  // {
  //   CGAL_assertion(clippingRectIsInitialized);
  //   Object o = CGAL::intersection(l, clippingRect);
  //   typedef CGAL_Segment_2 Segment_2;
  //   typedef CGAL_Point_2 Point_2;
  //   if(const Segment_2 *s = CGAL::object_cast<Segment_2>(&o)){
  //     return this->operator()(*s);
  //   } else if(const Point_2 *p = CGAL::object_cast<Point_2>(&o)){
  //     return QLineF(operator()(*p), operator()(*p));
  //   }
  //   return QLineF();
  // }

  // QPolygonF operator()(const CGAL_Triangle_2 &t) const
  // {
  //   QPolygonF qp;
  //   qp << operator()(t.vertex(0)) << operator()(t.vertex(1)) << operator()(t.vertex(2));
  //   return qp;
  // }


  // void operator()(std::list< CGAL_Point_2 >& p, const QPolygonF& qp) const
  // {
  //   for(int i = 0; i < qp.size(); i++){
  //     p.push_back(operator()(qp[i]));
  //   }
  // }

};

} // namespace Qt

#endif // CGAL_QT_CONVERTER_H