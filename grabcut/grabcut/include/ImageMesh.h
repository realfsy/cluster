#ifndef _IMAGE_MESH_H_
#define _IMAGE_MESH_H_

#include "Mesh/BaseMesh.h"
#include "Mesh/Edge.h"
#include "Mesh/Face.h"
#include "Mesh/HalfEdge.h"
#include "Mesh/Vertex.h"

#include "Mesh/Boundary.h"
#include "Mesh/Iterators.h"
#include "Parser/parser.h"
#include "graph.h"

namespace MeshLib
{
    class CImageVertex;
    class CImageEdge;
    class CImageFace;
    class CImageHalfEdge;

    /*! \brief CImageVertex class
     */
    class CImageVertex : public CVertex
    {
    public:
        /*! Constructor */
        CImageVertex() :
            m_index(0),
            m_mask(true),
            m_source_weight(0),
            m_sink_weight(0),
            m_rgb(0, 0, 0),
            m_pixel_value(0),
            m_node(NULL){ };
        //m_rgb(229.0 / 255.0, 162.0 / 255.0, 141.0 / 255.0) {};

        /*! Vertex index */
        int& idx() { return m_index; };

        /*! Vertex color */
        CPoint& rgb() { return m_rgb; };

        /*!
         *	Read vertex traits to vertex string
         */
        void _from_string();

        void _to_string();

        /*! Vertex source weight */
        float& source_weight() { return m_source_weight; };

        /*! Vertex sink weight */
        float& sink_weight() { return m_sink_weight; };

        /*! pixel value on each vertex */
        float& pixel_value(){ return m_pixel_value; };

        /*! graph node */
        MRF::Graph::node_id& graph_node() { return m_node; };

        bool& mask() { return m_mask; };
    protected:
        /*! Vertex index */
        int m_index;

        /*! Vertex color */
        CPoint m_rgb;

        /*! vertex source weight */
        float m_source_weight;

        /*! vertex sink weight */
        float m_sink_weight;

        /*! segmentation result, foreground true, background false */
        bool m_mask;

        /*! pixel value on each vertex */
        float m_pixel_value;

        /*! Graph Node*/
        MRF::Graph::node_id m_node;
    };

    inline void CImageVertex::_from_string()
    {
        CParser parser(m_string);
        for (std::list<CToken*>::iterator iter = parser.tokens().begin();
            iter != parser.tokens().end(); ++iter)
        {
            CToken* token = *iter;

            if (token->m_key == "rgb")
            {
                token->m_value >> m_rgb;
            }
        }
    }

    inline void CImageVertex::_to_string()
    {
        CParser parser(m_string);
        parser._removeToken("rgb");

        parser._toString(m_string);
        std::stringstream iss;

        iss << "rgb=(" << m_pixel_value << " " << m_pixel_value << " " << m_pixel_value << ")";

        if (m_string.length() > 0)
        {
            m_string += " ";
        }
        m_string += iss.str();
    }

    /*! \brief CImageEdge class
     */
    class CImageEdge : public CEdge
    {
    public:
        /*! Constructor */
        CImageEdge() : m_weight(0), m_length(0) {};

        /*!	Edge weight */
        float& weight() { return m_weight; };

        /*! Edge length */
        float& length() { return m_length; };

    protected:
        /*!	Edge weight */
        float m_weight;

        /*! Edge length */
        float m_length;

    };


    /*! \brief CImageFace class
     */
    class CImageFace : public CFace
    {
    public:
        /*! Constructor */
        CImageFace() { m_index = 0; };

        /*! face normal */
        CPoint& normal() { return m_normal; };

        /*! index */
        int& idx() { return m_index; };

    protected:
        /*! face normal */
        CPoint m_normal;

        /*! index */
        int m_index;
    };

    /*! \brief CImageHalfEdge class
     */
    class CImageHalfEdge : public CHalfEdge
    {
    public:
        /*!	CHarmonicHalfEdge constructor */
        CImageHalfEdge() {};

    protected:
    };

    /*! \brief CImageMesh class
     *
     *	Mesh class for cut graph algorithm
     *
     */
    template <typename V, typename E, typename F, typename H>
    class TImageMesh : public CBaseMesh<V, E, F, H>
    {
    public:
        typedef V CVertex;
        typedef E CEdge;
        typedef F CFace;
        typedef H CHalfEdge;

        typedef CBoundary<V, E, F, H>                   CBoundary;
        typedef CLoop<V, E, F, H>                       CLoop;

        typedef MeshVertexIterator<V, E, F, H>          MeshVertexIterator;
        typedef MeshEdgeIterator<V, E, F, H>            MeshEdgeIterator;
        typedef MeshFaceIterator<V, E, F, H>            MeshFaceIterator;
        typedef MeshHalfEdgeIterator<V, E, F, H>        MeshHalfEdgeIterator;

        typedef VertexVertexIterator<V, E, F, H>        VertexVertexIterator;
        typedef VertexEdgeIterator<V, E, F, H>          VertexEdgeIterator;
        typedef VertexFaceIterator<V, E, F, H>          VertexFaceIterator;
        typedef VertexInHalfedgeIterator<V, E, F, H>    VertexInHalfedgeIterator;
        typedef VertexOutHalfedgeIterator<V, E, F, H>   VertexOutHalfedgeIterator;

        typedef FaceVertexIterator<V, E, F, H>          FaceVertexIterator;
        typedef FaceEdgeIterator<V, E, F, H>            FaceEdgeIterator;
        typedef FaceHalfedgeIterator<V, E, F, H>        FaceHalfedgeIterator;
    };

    typedef TImageMesh<CImageVertex, CImageEdge, CImageFace, CImageHalfEdge> CImageMesh;
}

#endif //!_IMAGE_MESH_H_

