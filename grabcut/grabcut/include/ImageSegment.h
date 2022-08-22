#ifndef _IMAGE_SEGMENT_H_
#define _IMAGE_SEGMENT_H_
#include <opencv2/opencv.hpp>
#include "ImageMesh.h"
#include "coder_array.h"
namespace MeshLib
{
class CImageSegment
{
public:
    /*!
     *  Constructor.
     */
  CImageSegment(coder::array<float, 3U> &Y,
                coder::array<float, 3U> &pdf,
                coder::array<unsigned char,3U> &segbw,int dims);
    
    /*!
     *  Destructor.
     */
    ~CImageSegment();

    /*!
     *  Segment the input image.
     */
    void segment();

    /*!
     *  The reference of the input image.
     *  \return the reference of the input image.
     */

    /*!
     *  The reference of the segment image.
     *  \return the reference of the segment image.
     */

protected:
    /*
     * Compute the weights for vertices and edges.
     */
    void _prepare_weights();
    /*!
     *  Create a grid mesh for the input image.
     */
    void mx_create_grid_mesh();
    /*!
     *  Compute the distance between two pixel value.
     *  \param [in] pixel1: the first input pixel.
     *  \param [in] pixel2: the second input pixel.
     *  \return the distance.
     */
    float _distance(float pixel1, float pixel2);

protected:
    /*!
     *  The input image. 
     */

    /*!
     * The grid mesh of the image.
     */
    CImageMesh m_mesh;
    coder::array<float, 3U>& mx_image;
    unsigned char* mx_out_image;
    int mydims = 0;
    /*!
     *  the 2D array of vertex pointers.
     */
    CImageMesh::CVertex*** m_nodes;

    /*!
     *  The segment image. 
     */
    const float* capacity;
};
}
#endif //!_IMAGE_SEGMENT_H_