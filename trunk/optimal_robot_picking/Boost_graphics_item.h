/*
 *  A GraphicsItem derived class to enable the filling of the inside of a polygonal region. 
 *	Qt GraphicsItem class does not do filling by default (at least as of 4.8.5). 
 *
 *  Created on: Jan 30, 2015
 *  Author: Jingjin Yu
 */

#ifndef _O_ADV_GRAPHICS_ITEM_H_
#define _O_ADV_GRAPHICS_ITEM_H_



#include <QObject>
#include <QGraphicsItem>
#include <QColor>
#include <QPen>
#include <QBrush>
#include <QPainter>
#include "Boost_types.h"
#include "Boost_roadmap.h"
#include "Boost_converter.h"
#include "Boost_PainterOstream.h"
#include "GraphicsItem.h"
template<typename T>
class AdvancedGraphicsItem : public GraphicsItem{

public:
	bool m_bShowVertices;		// Draw vertices?
	bool m_bShowEdge;			// Draw edges?
	bool m_bFill;				// Fill the polygon?

	QPen m_vertexPen;			// Vertex pen
	QPen m_edgePen;				// Edge pen
	QBrush m_fillBrush;			// Fill brush

	// Need this constructor to take a T* pointer
	AdvancedGraphicsItem(T* p);

	// We need to overwrite paint method to do custom rendering such as filling the area. This
	// is not provided by default by QGraphicsItem in any other way.
	void paint( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

	//void modelChanged();

	QRectF boundingRect() const;
  
	inline 
  	const QPen& verticesPen() const
  	{
    	return m_vertexPen;
  	}

	inline
  	const QPen& edgesPen() const
  	{
    	return m_edgePen;
  	}
    inline             
  	void setVerticesPen(const QPen& pen)
  	{
    	m_vertexPen = pen;
  	}
	inline
  	void setEdgesPen(const QPen& pen)
  	{
    	m_edgePen = pen;
  	}
	inline
  	bool drawVertices() const
  	{
    	return m_bShowVertices;
  	}
	inline
  	void setDrawVertices(const bool b)
  	{
    	m_bShowVertices = b;
    	update();
  	}
	inline
  	bool drawEdges() const
  	{
    	return m_bShowEdge;
  	}
	inline
  	void setDrawEdges(const bool b)
  	{
    	m_bShowEdge = b;
    	update();
  	}

protected:
  	void updateBoundingBox();

  	T* poly;
  	QPainter* m_painter;
  	PainterOstream painterostream;
	Qt::Converter convert;
  	typename Point_2 p;
  	QRectF bounding_rect;

 
};

template<typename T> inline 
AdvancedGraphicsItem<T>::AdvancedGraphicsItem(T* p):
		poly(p),
		painterostream(0),
		m_bShowVertices(false),
		m_bShowEdge(true), 
		m_bFill(true),
		m_vertexPen(QPen(Qt::red, 0.025, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin)),
		m_edgePen(QPen(Qt::red, 0.025, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin)),
		m_fillBrush(QColor(Qt::gray))
		{
			updateBoundingBox();
		}


// We need to overwrite paint method to do custom rendering such as filling the area. This
// is not provided by default by QGraphicsItem in any other way.
		
template<class T> inline  void AdvancedGraphicsItem<T>
::paint( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget){

	// Paint the edges
	if(m_bShowEdge){
		painter->setPen(m_edgePen);
		painterostream  = PainterOstream(painter);
		 
	//	boost::geometry::ring_type<Polygon_2>::type somering = poly->outer();
	//	boost::geometry::ring_type<Polygon_2>::type::iterator it;
		std::vector<Segment_2> outer_edge_list;
		int i;
		for(i = 0;i < poly->outer().size()-1;i++){
			Segment_2 seg(poly->outer()[i], poly->outer()[i+1]);
			outer_edge_list.push_back(seg);
		}
	//	for(it = somering.begin(); (it+1) != somering.end(); )
	//	{
	//		Segment_2 seg((*it), (*(++it)));


			//std::cout<<(*it).get<0>()<<","<<(*it).get<1>()<<std::endl;
	//		outer_edge_list.push_back(seg);
	//	}
		std::vector<Segment_2>::iterator eit;
		for(eit = outer_edge_list.begin(); eit != outer_edge_list.end(); ++eit)
		{

			painter->drawLine(convert((*eit).first), convert((*eit).second));
			//painterostream << *eit;
		}
		
	}

	// Paint vertices if needed and obtain the fill area
	if(m_bShowVertices || m_bFill){
		Qt::Converter convert;
		painter->setPen(m_vertexPen);
		QMatrix matrix = painter->matrix();
		painter->resetMatrix();
		QPainterPath path;
		boost::geometry::ring_type<Polygon_2>::type somering = poly->outer();
		boost::geometry::ring_type<Polygon_2>::type::iterator it;
		for(it = somering.begin(); it != somering.end(); it++)
		{ 
				QPointF point = matrix.map(convert(*it));
				// Draw vertices here as needed
				if(m_bShowVertices){
					painter->drawPoint(point);

				}
				if(it == somering.begin()){
					path.moveTo(point.rx(), point.ry());
				}
				else{
					path.lineTo(point.rx(), point.ry());
				}
		}
		path.closeSubpath();
		// Fill the area if needed
		if(m_bFill){
			painter->fillPath(path, QBrush(m_fillBrush));
		}
	}
}

template <typename T>
inline
QRectF 
AdvancedGraphicsItem<T>::boundingRect() const
{
	return bounding_rect;
}

template <typename T>
void 
AdvancedGraphicsItem<T>::updateBoundingBox()
{
  Qt::Converter convert;
  //prepareGeometryChange();
  if(poly->outer().size() == 0){
    return;
  }
  bounding_rect = convert(bg::return_envelope<Box_2>(*poly));
}


#endif /* _O_ADV_GRAPHICS_ITEM_H_ */
