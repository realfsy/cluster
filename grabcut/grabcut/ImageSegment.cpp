#include "ImageSegment.h"

namespace MeshLib {
std::string type2str(int type)
{
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
  case CV_8U:
    r = "8U";
    break;
  case CV_8S:
    r = "8S";
    break;
  case CV_16U:
    r = "16U";
    break;
  case CV_16S:
    r = "16S";
    break;
  case CV_32S:
    r = "32S";
    break;
  case CV_32F:
    r = "32F";
    break;
  case CV_64F:
    r = "64F";
    break;
  default:
    r = "User";
    break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

CImageSegment::CImageSegment(coder::array<float, 3U> &Y,
                             coder::array<float, 3U> &pdf,
                             coder::array<unsigned char, 3U> &segbw, int dims)
    : mx_image(Y), capacity((float *)pdf.data()), mydims(dims)
{
  // 1. read image
  auto aa = mx_image.size();
 
    segbw.set_size(aa[0], aa[1], aa[2]);

  mx_out_image = (unsigned char *)segbw.data();

  if (dims == 2) {
    int ni = aa[0], nj = aa[1];
    int stepi = 1, stepj = aa[0];
    int totaln = aa[0] * aa[1];
    // mexPrintf("size:%d %d %d %d\n", aa[1], aa[0], sizeof(CV_64FC1), totaln);

    // memcpy((void*)(m_image.data), (void*)mxGetData(mximage), totaln *
    // sizeof(CV_64FC1));
    int rows = ni;
    int cols = nj;

    // 2. allocate the memory for m_nodes
    m_nodes = new CImageMesh::CVertex **[rows];
    if (m_nodes == NULL) {
      std::cerr << "Memory allocatin error!" << std::endl;
      return;
    }
    for (int i = 0; i < rows; i++) {
      m_nodes[i] = new CImageMesh::CVertex *[cols];
      if (m_nodes[i] == NULL) {
        std::cerr << "Memory allocatin error!" << std::endl;
        return;
      }
    }

    // 3. create grid mesh
    mx_create_grid_mesh();
    float *p0 = (float *)mx_image.data();
    // 4. set pixel value for each node
    for (int j = 0; j < nj; j++) {
      for (int i = 0; i < ni; i++) {
        int idx = i * stepi + j * stepj;
        m_nodes[i][j]->pixel_value() = p0[idx];
      }
    }
  }
}

CImageSegment::~CImageSegment()
{
}

void CImageSegment::segment()
{

  if (mydims == 2) {
    this->_prepare_weights();

    std::cout << "Start segmentation " << std::endl;

    CImageMesh *pMesh = &m_mesh;
    MRF::Graph *g = new MRF::Graph();

    for (CImageMesh::MeshVertexIterator viter(pMesh); !viter.end(); viter++) {
      CImageMesh::CVertex *pV = *viter;
      pV->graph_node() = g->add_node();
    }

    for (CImageMesh::MeshVertexIterator viter(pMesh); !viter.end(); viter++) {
      CImageMesh::CVertex *pV = *viter;
      g->set_tweights(pV->graph_node(), pV->source_weight(), pV->sink_weight());
    }

    for (CImageMesh::MeshEdgeIterator eiter(pMesh); !eiter.end(); eiter++) {
      CImageMesh::CEdge *pE = *eiter;
      CImageMesh::CVertex *pV1 = pMesh->edgeVertex1(pE);
      CImageMesh::CVertex *pV2 = pMesh->edgeVertex2(pE);
      g->add_edge(pV1->graph_node(), pV2->graph_node(), pE->weight(),
                  pE->weight());
    }

    MRF::Graph::flowtype flow = g->maxflow();

    for (CImageMesh::MeshVertexIterator viter(pMesh); !viter.end(); viter++) {
      CImageMesh::CVertex *pV = *viter;
      if (g->what_segment(pV->graph_node()) == MRF::Graph::SOURCE) {
        pV->mask() = true;
      } else {
        pV->mask() = false;
      }
    }
    delete g;

    std::cout << "Finish segmentation" << std::endl;

    // extract the result into segment image.
    for (CImageMesh::MeshVertexIterator viter(pMesh); !viter.end(); viter++) {
      CImageMesh::CVertex *pV = *viter;
      int idx = pV->id() - 1;
      if (pV->mask() == false) // on background
        mx_out_image[idx] = 0;
      else
        mx_out_image[idx] = 1;
    }
    return;
  }
  const int *dims = mx_image.size();
  int ni, nj, nk;
  ni = dims[0];
  nj = dims[1];
  nk = dims[2];
  float d1, d2, d3, d4;
  int i1, i2, i3, i4;
  std::array<int, 3> offset = {1, ni, ni * nj};
  int s1, s2, s3, li, lj, lk;
  std::array<int, 3> temp;
  MRF::Graph *g = new MRF::Graph();
  std::vector<MRF::Graph::node_id> nodes(ni * nj * nk);
  for (auto &it : nodes)
    it = g->add_node();
  i1 = 0;
  for (auto &it : nodes) {
    g->set_tweights(it, capacity[i1], 1 - capacity[i1]);
    i1++;
  }
  auto get_offset = [&li, &lj, &lk, &offset]() -> int {
    return lk * offset[2] + lj * offset[1] + li;
  };
  float *image = (float *)mx_image.data();
  int count = (ni - 1) * nj * nk + ni * (nj - 1) * nk + ni * nj * (nk - 1);
  MRF::Graph::node_id **nodep1 = new MRF::Graph::node_id *[count];
  MRF::Graph::node_id **nodep2 = new MRF::Graph::node_id *[count];
  float *disp = new float[count];
  count = 0;
  float mmin = 10;
  float mmax = -1;
  MRF::Graph::node_id **p1 = nodep1, **p2 = nodep2;
  float *fp = disp;
  for (li = 0; li < ni - 1; li++)
    for (lj = 0; lj < nj; lj++)
      for (lk = 0; lk < nk; lk++) {
        s1 = get_offset();
        s2 = s1 + offset[0];
        d1 = _distance(image[s1], image[s2]);
        *(p1++) = nodes.data() + s1;
        *(p2++) = nodes.data() + s2;
        mmax = d1 > mmax ? d1 : mmax;
        mmin = d1 < mmin ? d1 : mmin;
        *(fp++) = d1;
        count++;
      }
  for (li = 0; li < ni; li++)
    for (lj = 0; lj < nj - 1; lj++)
      for (lk = 0; lk < nk; lk++) {
        s1 = get_offset();
        s2 = s1 + offset[1];
        d1 = _distance(image[s1], image[s2]);
        *(p1++) = nodes.data() + s1;
        *(p2++) = nodes.data() + s2;
        mmax = d1 > mmax ? d1 : mmax;
        mmin = d1 < mmin ? d1 : mmin;
        *(fp++) = d1;
        count++;
      }
  for (li = 0; li < ni; li++)
    for (lj = 0; lj < nj; lj++)
      for (lk = 0; lk < nk - 1; lk++) {
        s1 = get_offset();
        s2 = s1 + offset[2];
        d1 = _distance(image[s1], image[s2]);
        *(p1++) = nodes.data() + s1;
        *(p2++) = nodes.data() + s2;
        mmax = d1 > mmax ? d1 : mmax;
        mmin = d1 < mmin ? d1 : mmin;
        *(fp++) = d1;
        count++;
      }
  count = (ni - 1) * nj * nk + ni * (nj - 1) * nk + ni * nj * (nk - 1);
  float f1 = 1. / (mmax - mmin);
  p1 = nodep1;
  p2 = nodep2;
  fp = disp;
  for (int i = 0; i < count; i++, fp++, p1++, p2++) {
    *fp = ((*fp) - mmin) * f1;
    g->add_edge(**p1, **p2, *fp, *fp);
  }

  delete[] nodep1;
  delete[] nodep2;
  delete[] disp;
  MRF::Graph::flowtype flow = g->maxflow();
  i1 = 0;
  for (auto &it : nodes) {
    mx_out_image[i1++] = (g->what_segment(it) == MRF::Graph::SOURCE) ? 1 : 0;
  }

  delete g;
}

void CImageSegment::mx_create_grid_mesh()
{
  auto aa = mx_image.size();
  int dims = mydims;

  int ni = aa[0], nj = aa[1];
  int stepi = 1, stepj = aa[0];

  for (int i = 0; i < ni; i++) {
    for (int j = 0; j < nj; j++) {
      int vidx = i * stepi + j * stepj;
      int vid = vidx + 1;
      CImageMesh::CVertex **pVertex = &m_nodes[i][j];
      *pVertex = m_mesh.createVertex(vid);
      (*pVertex)->point() = CPoint(j / (float)nj, i / (float)ni, 0);
    }
  }

  int offset[4][2] = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  for (int i = 0; i < ni - 1; i++) {
    for (int j = 0; j < nj - 1; j++) {
      int fid = i * (ni - 1) + j + 1;

      std::vector<CImageMesh::CVertex *> verts;

      for (int k = 0; k < 4; k++) {
        int row = i + offset[k][0];
        int col = j + offset[k][1];
        verts.push_back(m_nodes[row][col]);
      }
      m_mesh.createFace(verts, fid);
    }
  }

  m_mesh.labelBoundary();
}

float CImageSegment::_distance(float pixel1, float pixel2)
{

  float d = pixel1 - pixel2;

  return exp(-d * d * 9);
}

void CImageSegment::_prepare_weights()
{
  CImageMesh *pMesh = &m_mesh;

  for (CImageMesh::MeshEdgeIterator eiter(pMesh); !eiter.end(); eiter++) {
    CImageMesh::CEdge *pEdge = *eiter;
    CImageMesh::CVertex *pS = pMesh->edgeVertex1(pEdge);
    CImageMesh::CVertex *pT = pMesh->edgeVertex2(pEdge);

    float pixel1 = pS->pixel_value();
    float pixel2 = pT->pixel_value();
    pEdge->weight() = _distance(pixel1, pixel2);
  }

  for (CImageMesh::MeshVertexIterator viter(pMesh); !viter.end(); viter++) {
    CImageMesh::CVertex *pV = *viter;
    int idx = pV->id() - 1;
    pV->source_weight() = capacity[idx];
    pV->sink_weight() = 1 - capacity[idx];
  }
}

} // namespace MeshLib