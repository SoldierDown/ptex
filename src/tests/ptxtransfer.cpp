/*
* Copyright Disney Enterprises, Inc.  All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License
* and the following modification to it: Section 6 Trademarks.
* deleted and replaced with:
*
* 6. Trademarks. This License does not grant permission to use the
* trade names, trademarks, service marks, or product names of the
* Licensor and its affiliates, except as required for reproducing
* the content of the NOTICE file.
*
* You may obtain a copy of the License at
* http://www.apache.org/licenses/LICENSE-2.0
*/

#include <Ptexture.h>
#include <PtexUtils.h>
#include <PtexHalf.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <time.h>
#include "KdTree.h"
#include <iomanip>
#include <fstream>
#include <sstream>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
typedef CGAL::Simple_cartesian<float> K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
//typedef CGAL::Polyhedron_items_with_id_3<K> Polyhedron;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;

class Vec3f
{
    
 public:
    float p[3];
    inline Vec3f()
    { p[0]=0.0f; p[1]=0.0f; p[2]=0.0f; }
    
    inline explicit Vec3f(float x)
    { p[0]=x; p[1]=x; p[2]=x; }
    
    inline Vec3f(float x, float y, float z)
    { p[0]=x; p[1]=y; p[2]=z; }
    
    inline Vec3f(const float *f)
    { p[0]=f[0]; p[1]=f[1]; p[2]=f[2]; }
    
    inline Vec3f(const double *d)
    { p[0]=d[0]; p[1]=d[1]; p[2]=d[2]; }
    
    inline float* getValue()
    { return p; }
    
    inline const float* getValue() const
    { return p; }
    
    inline float length() const
    { return std::sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]); }

    inline float& operator[](int n)
    { return p[n]; }
    
    inline const float& operator[](int n) const
    { return p[n]; }
    
    inline Vec3f operator+(const Vec3f &v) const
    { return Vec3f(p[0]+v.p[0], p[1]+v.p[1], p[2]+v.p[2]); }
    
    inline Vec3f operator-(const Vec3f &v) const
    { return Vec3f(p[0]-v.p[0], p[1]-v.p[1], p[2]-v.p[2]); }
    
    inline Vec3f operator-() const
    { return Vec3f(-p[0], -p[1], -p[2]); }
    
    inline Vec3f operator*(float f) const
    { return Vec3f(p[0]*f, p[1]*f, p[2]*f); } const
    
    inline Vec3f operator/(float f)
    { return Vec3f(p[0]/f, p[1]/f, p[2]/f); }
};

struct Quad {
    Vec3f v[4];
    Quad(const Vec3f& _v0, const Vec3f& _v1, const Vec3f& _v2, const Vec3f& _v3)
    {
        v[0] = _v0;
        v[1] = _v1;
        v[2] = _v2;
        v[3] = _v3;
    }
};

class PointCloud
{
 private:
    unsigned char* _values;
    int _totalpoints;
    int _datasize;
    KdTree<3> _tree;
        
 public:
    PointCloud(int datasize) : _values(0), _totalpoints(0), _datasize(datasize) {}
    ~PointCloud() { if (_values) delete [] _values; }
    
    bool setSize(int numpts)
    {
        _totalpoints = numpts;
        if (_values) delete [] _values;
        _values = new unsigned char[_totalpoints*_datasize];
        _tree.allocatePoints(_totalpoints);
        return true;
    }
    
    int size() { return _totalpoints; }
    
    void sortTree() { _tree.sort(); }
    
    void store(int i, const float position[3], const void* data)
    {
        if (i>=_totalpoints) return;
        _tree.setPoint(i, (float*)&position[0]);
        memcpy(&_values[i*_datasize], data, _datasize);
    }
    
    bool find(const float position[3], void* data, float distance = 1e9f)
    {
        int index = _tree.findNearest(&position[0], distance);
        if (index<0) return false;
        memcpy(data, &_values[_tree.id(index)*_datasize], _datasize);
        return true;
    }
};


class Mesh
{
 public:
    std::vector<Vec3f> verts;
    std::vector< std::vector<int> > faceVertIndices;
    int numFaces() const { return faceVertIndices.size(); }
    int numVerts() const { return verts.size(); }
    
    inline bool evaluate(int faceid, float u, float v, Vec3f& p, int resU = 0, int resV = 0)
    {
        if (faceid>=int(faceVertIndices.size())) return false;
        
        std::vector<int>& vind = faceVertIndices[faceid];
        
        if (vind.size()==4)
        {
            p = verts[vind[0]]*(1-u)*(1-v) +
                verts[vind[1]]*u*(1-v) +
                verts[vind[2]]*u*v +
                verts[vind[3]]*(1-u)*v;
            return true;
        }
        
        if (vind.size()==3)
        {
            if (resU>0 && resV>0)
            {
                if (u+v<=1)
                {
                    u = u - 1.0f/(resU*6);
                    v = v - 1.0f/(resV*6);
                }
                else
                {
                    u = 1 + 1.0f/(resU*6) - u;
                    v = 1 + 1.0f/(resV*6) - v;
                }
            }
            p = verts[vind[0]] +
                (verts[vind[1]]-verts[vind[0]])*u + 
                (verts[vind[2]]-verts[vind[0]])*v;
            return true;
        }
        
        return false;
    }
    
    float faceTexelSize(int faceid, int resU, int resV)
    {
        std::vector<int>& vind = faceVertIndices[faceid];
        if (vind.size()==4)
        {
            return std::min(
                std::min((verts[vind[1]] - verts[vind[0]]).length(),
                         (verts[vind[2]] - verts[vind[3]]).length()) / resU,
                std::min((verts[vind[3]] - verts[vind[0]]).length(),
                         (verts[vind[2]] - verts[vind[1]]).length()) / resV);
                
        }
        else if (vind.size()==3)
        {
            return std::min( std::min((verts[vind[1]] - verts[vind[0]]).length(),
                         (verts[vind[2]] - verts[vind[1]]).length()),
                         (verts[vind[0]] - verts[vind[2]]).length()) / resU;
        }
        
        return 0;
    }
    
    Ptex::Res calculateFaceResolution(int faceid, float texelSize)
    {
        std::vector<int>& vind = faceVertIndices[faceid];
        if (vind.size()==4)
        {
            uint32_t resU = PtexUtils::floor_log2(std::max((verts[vind[1]] - verts[vind[0]]).length(),
                            (verts[vind[2]] - verts[vind[3]]).length()) / texelSize);
            uint32_t resV = PtexUtils::floor_log2(std::max((verts[vind[3]] - verts[vind[0]]).length(),
                            (verts[vind[2]] - verts[vind[1]]).length()) / texelSize);
            
            return Ptex::Res(resU,resV);
                
        }
        else if (vind.size()==3)
        {
            float len = std::max(std::max((verts[vind[1]] - verts[vind[0]]).length(),
                                 (verts[vind[2]] - verts[vind[1]]).length()),
                                 (verts[vind[0]] - verts[vind[2]]).length());
            
            uint32_t resU = PtexUtils::floor_log2(len / texelSize);
            return Ptex::Res(resU,resU);
        }
        
        return Ptex::Res(1,1);
    }
    
    bool getMeshData(PtexTexture* r)
    {
        if (!r) return false;

        PtexPtr<PtexMetaData> meta(r->getMetaData());
        if (!meta) return false;

        if (meta->numKeys()<3)
        {
            std::cerr << "Geometry meta data not found in Ptex file\n";
            return false;
        }
        
        const float* vp;
        const int *fvi, *fvc;
        int vertcount, indexcount, facevertcount;

        meta->getValue("PtexFaceVertCounts",fvc,facevertcount);
        meta->getValue("PtexVertPositions",vp,vertcount);
        meta->getValue("PtexFaceVertIndices",fvi,indexcount);

        if (facevertcount==0)
        {
            std::cerr << "Missing meta data : PtexFaceVertCounts\n";
            return false;
        }

        if (vertcount==0)
        {
            std::cerr << "Missing meta data : PtexVertPositions\n";
            return false;
        }

        if (indexcount==0)
        {
            std::cerr << "Missing meta data : PtexFaceVertIndices\n";
            return false;
        }

        verts.resize(vertcount/3);
        for (size_t i=0; i<verts.size(); i++) verts[i] = Vec3f(vp[i*3],vp[i*3+1],vp[i*3+2]);

        int vi = 0;
        for (int fi=0; fi<facevertcount; fi++)
        {
            int nverts = fvc[fi];
            if (r->meshType()==Ptex::mt_quad && nverts!=4)
            {
                // Split n-sided polygon into quads

                Vec3f cp = Vec3f(0,0,0); // center of polygon
                int vib = verts.size();

                // Find edge midpoints and polygon center
                for (int i=0; i<nverts; i++)
                {
                    Vec3f& v = verts[fvi[vi+i]];
                    cp = cp + v;
                    verts.push_back((v+verts[fvi[vi+(i+1)%nverts]])/2);
                }
                verts.push_back(cp/nverts);

                // Add new quads using edge midpoints and polygon center
                for (int i=0; i<nverts; i++)
                {
                    std::vector<int> v;
                    v.push_back(fvi[vi+i]);
                    v.push_back(vib+i);
                    v.push_back(verts.size()-1);
                    v.push_back(vib+(i+nverts-1)%nverts);
                    faceVertIndices.push_back(v);
                }

                vi += nverts;
            }
            else if (r->meshType()==Ptex::mt_triangle && nverts!=3)
            {
                std::cerr << "Illegal vertex count for triangular Ptex\n";
                return false;
            }
            else
            {
                std::vector<int> v;
                for (int i=0; i<nverts; i++) v.push_back(fvi[vi++]);
                faceVertIndices.push_back(v);
            }
        }

        if (int(faceVertIndices.size())!=r->numFaces())
        {
            std::cerr << "Face count mismatch\n";
            return false;
        }

        return true;
    }
};


class PixelBuffer
{
    int _numChannels;
    Ptex::DataType _dataType;
    void* _data;
    
 public:
    PixelBuffer(Ptex::DataType dataType, int numChannels, int numPixels) : _numChannels(numChannels), _dataType(dataType), _data(0)
    {
        int numValues = numPixels*_numChannels;
        
        switch (_dataType)
        {
            case Ptex::dt_uint8:  _data = new uint8_t[numValues];  break;
            case Ptex::dt_float:  _data = new float[numValues];    break;
            case Ptex::dt_half:   _data = new PtexHalf[numValues]; break;
            case Ptex::dt_uint16: _data = new uint16_t[numValues]; break;
        }
    }
    
    ~PixelBuffer()
    {
        switch (_dataType)
        {
            case Ptex::dt_uint8:  delete [] ((uint8_t*)_data);  break;
            case Ptex::dt_float:  delete [] ((float*)_data);    break;
            case Ptex::dt_half:   delete [] ((PtexHalf*)_data); break;
            case Ptex::dt_uint16: delete [] ((uint16_t*)_data); break;
        }
    }

    inline void writePixel(float* val, int pixelIndex)
    {
        int d = _numChannels*pixelIndex;
        
        switch (_dataType)
        {
        case Ptex::dt_uint8:
            for (int i=0; i<_numChannels; i++) ((uint8_t*)_data)[d+i] = uint8_t(val[i]*255);
            break;
        case Ptex::dt_float:
            for (int i=0; i<_numChannels; i++) ((float*)_data)[d+i] = val[i];
            break;
        case Ptex::dt_half:
            for (int i=0; i<_numChannels; i++) ((PtexHalf*)_data)[d+i] = val[i];
            break;
        case Ptex::dt_uint16:
            for (int i=0; i<_numChannels; i++) ((uint16_t*)_data)[d+i] = uint16_t(val[i]*65535);
        }
    }
    
    void* getData() { return _data; }
    
    void* getPixel(int pixelIndex)
    { return (unsigned char*)_data+pixelIndex*_numChannels*Ptex::DataSize(_dataType); }
};

std::vector<Vec3f> readRootVerts(std::string filename)
{
    std::vector<Vec3f> vertices;

    std::ifstream objFile(filename);
    if (!objFile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return vertices;
    }

    std::string line;
    while (std::getline(objFile, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            float x, y, z;
            iss >> x >> y >> z;
            vertices.emplace_back(x, y, z);
        }
    }

    objFile.close();
    return vertices;
}

std::vector<Quad> constructCGALQuads(const Mesh& mesh)
{
    std::vector<Quad> quads;
    std::vector<Vec3f> vertices = mesh.verts;
    for (int fid = 0; fid < mesh.faceVertIndices.size(); ++fid)
    {
        const std::vector<int>& face = mesh.faceVertIndices[fid];
        quads.push_back(Quad(vertices[mesh.faceVertIndices[fid][0]], vertices[mesh.faceVertIndices[fid][1]], vertices[mesh.faceVertIndices[fid][2]], vertices[mesh.faceVertIndices[fid][3]]));
    }
    return quads;
}


float dot(Vec3f a, Vec3f b)
{
    return a.p[0] * b.p[0] + a.p[1] * b.p[1] + a.p[2] * b.p[2];
}

Vec3f cross(Vec3f a, Vec3f b)
{
    Vec3f result;
    result.p[0] = a.p[1] * b.p[2] - a.p[2] * b.p[1];
    result.p[1] = a.p[2] * b.p[0] - a.p[0] * b.p[2];
    result.p[2] = a.p[0] * b.p[1] - a.p[1] * b.p[0];
    return result;
}

float pointToPlaneDistance(const Vec3f& P, const Vec3f& normal, const Vec3f& pointOnPlane) {
    Vec3f disp = P - pointOnPlane;
    return std::abs(dot(disp, normal) / normal.length());
}

float distanceSquared(const Vec3f& p1, const Vec3f& p2) {
    return (p1 - p2).length();
}

bool pointInQuad(const Vec3f& queryPoint, const Quad& quad)
{
    return false;
}

Point Vec3f2Point(const Vec3f& p)
{
    return Point(p.p[0], p.p[1], p.p[2]);
}


std::vector<int> getPaintedFaces(const std::string& filename, const Mesh& mesh, const PtexPtr<PtexTexture>& fromPtex)
{
    const float radius = 1.f;
    std::vector<int> white_faces(mesh.numFaces(), 0);
    std::vector<Vec3f> root_vertices = readRootVerts(filename);
    std::vector<Quad> quads = constructCGALQuads(mesh);

    Polyhedron polyhedron;
    for (int qid = 0; qid < quads.size(); ++qid)
    {
        Point v0 = Vec3f2Point(quads[qid].v[0]);
        Point v1 = Vec3f2Point(quads[qid].v[1]);
        Point v2 = Vec3f2Point(quads[qid].v[2]);
        Point v3 = Vec3f2Point(quads[qid].v[3]);
        polyhedron.make_tetrahedron(v0, v1, v2, v3);
    }
    int i = 0;
    for (Polyhedron::Face_iterator facet = polyhedron.facets_begin(); facet != polyhedron.facets_end(); ++facet) {
        facet->id() = i++;
    }

    // Build the AABB tree for efficient spatial search
    Tree tree(CGAL::faces(polyhedron).first, CGAL::faces(polyhedron).second, polyhedron);

    for(int rid=0;rid<root_vertices.size();++rid)
    {
        Point query_point = Vec3f2Point(root_vertices[rid]);
        Point_and_primitive_id pp = tree.closest_point_and_primitive(query_point);
        Point p = pp.first;
        Polyhedron::Face_handle f = pp.second;
        int id = f->id()/4;
        if (id >= 0)
        {
            white_faces[id] = 1;
        }
    }

    bool to_continue = false;
    while(to_continue)
    {
        std::vector<int> white_faces_bak = white_faces;
        for(int fid=0;fid<fromPtex->numFaces();++fid)
        {
            const Ptex::FaceInfo& f = fromPtex->getFaceInfo(fid);
            //if (white_faces_bak[fid])
            //{
            //    for (int adj = 0; adj < 4; ++adj)
            //    {
            //        int fn = f.adjfaces[adj];
            //        if (fn >= 0)
            //        {
            //            white_faces[fn] = 1;
            //        }
            //    }
            //}
            int numActivated = 0;            
            for(int adj=0;adj<4;++adj)
            {
                if(white_faces_bak[f.adjfaces[adj]])
                {
                    ++numActivated;
                }
            }
            if(numActivated >= 3)
            {
                white_faces[fid] = 1;
            }
        }
        if(white_faces == white_faces_bak)
        {
            to_continue = false;
        }
    }
    
    //white_faces = std::vector<int>(fromPtex->numFaces(), 1);
    return white_faces;
}

void imageMirrorRows(unsigned char* data, int w, int h, int datasize)
{
    unsigned char* tmp = new unsigned char[datasize];
    int halfw = w/2;
    
    for (int pix=0; pix<h*halfw; pix++)
    {
        int y = pix/halfw;
        int x = pix-y*halfw;
        unsigned char* row = data+y*w*datasize;
        memcpy(tmp, row+x*datasize, datasize);
        memcpy(row+x*datasize, row+(w-1-x)*datasize, datasize);
        memcpy(row+(w-1-x)*datasize, tmp, datasize);
    }
    
    delete [] tmp;
}

void imageRotate(unsigned char* data, int w, int h, int datasize, int rotations)
{
    if (rotations!=1 && rotations!=3) return;
    
    unsigned char* newdata = new unsigned char[w*h*datasize];
    
    if (rotations==1)
    {
        for (int y=0; y<h; y++)
            for (int x=0; x<w; x++)
                memcpy(newdata+(y+(w-1-x)*h)*datasize, data+(x+y*w)*datasize, datasize);
    }
    else if (rotations==3)
    {
        for (int y=0; y<h; y++)
            for (int x=0; x<w; x++)
                memcpy(newdata+((h-1-y)+x*h)*datasize, data+(x+y*w)*datasize, datasize);
    }
    memcpy(data, newdata, datasize*w*h);
    delete [] newdata;
}

bool transferPtex(std::string input, std::string output, std::string roots, float searchDist = -1, float matchDist = -1)
{
    time_t timer1;
    time(&timer1);
    
    Ptex::String ptexError;
    
    PtexPtr<PtexTexture> fromPtex(PtexTexture::open(input.c_str(), ptexError, true));
    if (!fromPtex)
    {
        std::cerr << ptexError << "\n";
        return false;
    }

    Mesh fromMesh;
    if (!fromMesh.getMeshData(fromPtex))
    {
        return false;
    }
    std::vector<int> white_faces = getPaintedFaces(roots, fromMesh, fromPtex);
    Ptex::MeshType meshType = fromPtex->meshType();
    Ptex::DataType dataType = fromPtex->dataType();
    int targetAlpha = fromPtex->alphaChannel();
    
    int numchan = fromPtex->numChannels();
    int datasize = numchan*Ptex::DataSize(fromPtex->dataType());
    PtexMetaData* meta = fromPtex->getMetaData();
    
    // Find total number of texels
    int totalTexels = 0;
    std::vector<int> baseIndex(fromPtex->numFaces());
    for (int faceid=0; faceid<fromPtex->numFaces(); faceid++)
    {
        baseIndex[faceid] = totalTexels;
        totalTexels += fromPtex->getFaceInfo(faceid).res.size();
    }
    float avgTexelSize = 0;
    
    // Create point cloud
    PointCloud fullPts(datasize);
    fullPts.setSize(totalTexels);
    
    // Sparse point cloud with four samples for each face, for face matching
    PointCloud facePts(sizeof(int)*2);
    facePts.setSize(fromPtex->numFaces()*4);
    
    // Populate point cloud using input Ptex data
    Vec3f pmin(1000.f);
    Vec3f pmax(-1000.f);
    
    for (int faceid=0; faceid<fromPtex->numFaces(); faceid++)
    {
         //std::cout << "Reading faces: " << faceid+1 << " / " << fromPtex->numFaces() << std::endl;
        
        const Ptex::FaceInfo& f = fromPtex->getFaceInfo(faceid);
        
        unsigned char* data = new unsigned char[f.res.size()*datasize];
        fromPtex->getData(faceid, data, 0);
        // Add points for each texel in input face
// #pragma omp parallel for
        for (int pix=0; pix<int(f.res.size()); pix++)
        {
            int uPixel = pix%f.res.u();
            int vPixel = pix/f.res.u();
            float u = (float(uPixel)+0.5f)/f.res.u();
            float v = (float(vPixel)+0.5f)/f.res.v();
            Vec3f p;
            fromMesh.evaluate(faceid,u,v,p,f.res.u(),f.res.v());
            for(int axis=0;axis<3;++axis)
            {
                pmin.p[axis] = std::min(pmin.p[axis], p.p[axis]);
                pmax.p[axis] = std::max(pmax.p[axis], p.p[axis]);
            }
            fullPts.store(baseIndex[faceid] + pix, p.getValue(), data+pix*datasize);
        }
        delete [] data;
        
        float s = fromMesh.faceTexelSize(faceid, f.res.u(), f.res.v());
        avgTexelSize += s;
    }
    
    if (fromPtex->meshType()==Ptex::mt_quad)
    {
// #pragma omp parallel for
        for (int faceid=0; faceid<fromPtex->numFaces(); faceid++)
        {
            // Add four samples for each input face to sparse point cloud
            Vec3f p;
            int faceData[] = { faceid,0 };
            for (int i=0; i<4; i++)
            {
                float u = (i==0 || i==3) ? 0.25 : 0.75;
                float v = (i<2) ? 0.25 : 0.75;
                faceData[1] = i;
                fromMesh.evaluate(faceid, u, v, p, 2, 2);
                facePts.store(faceid*4+i, p.getValue(), &faceData);
            }
        }
    }
    avgTexelSize /= fromPtex->numFaces();
    
     std::cout << "\n  Sorting texels...\n";
    
    fullPts.sortTree();
    if (fromPtex->meshType()==Ptex::mt_quad) facePts.sortTree();
    
    if (searchDist<0)
    {
        searchDist = avgTexelSize*40;
         std::cout << "  Calculating a search distance: " << searchDist << "\n";
    }
    if (matchDist<0) matchDist = avgTexelSize;
    
    // Write output data
    PtexPtr<PtexWriter> w(PtexWriter::open(output.c_str(), meshType, dataType,
                          numchan, targetAlpha, fromMesh.numFaces(), ptexError));

    float* errval = new float[numchan];
    for (int i=0; i<numchan; i++) errval[i] = 1;
    
    int copied = 0;
        
    for (int faceid=0; faceid<fromMesh.numFaces(); faceid++)
    {
        //std::cout << "  Writing faces: " << faceid+1 << " / " << fromMesh.numFaces() << "\r" << std::flush;
        
        Ptex::FaceInfo f = fromPtex->getFaceInfo(faceid);
        
        // Check for direct face copy
        int fromFace = -1;
        int faceData[4][2];
        
        if (matchDist>0 && fromPtex->meshType()==Ptex::mt_quad)
        {
            Vec3f pos;
            for (int i=0; i<4; i++)
            {
                float u = (i==0 || i==3) ? 0.25 : 0.75;
                float v = (i<2) ? 0.25 : 0.75;
                if (!fromMesh.evaluate(faceid,u,v,pos,2,2)) { fromFace = -1; break; }
                if (!facePts.find(pos.getValue(), &faceData[i], matchDist)) { fromFace = -1; break; }
                if (i==0) fromFace = faceData[i][0];
                else if (faceData[i][0]!=fromFace) { fromFace = -1; break; }
            }
        }
        
        if (fromFace>=0)
        {
            bool match = true;
            bool flipU = false;
            bool flipV = false;
            int rotations = 0;
            
            const int& v0 = faceData[0][1];
            const int& v1 = faceData[1][1];
            const int& v2 = faceData[2][1];
            const int& v3 = faceData[3][1];
            
            if (true)
            {
                f.res = fromPtex->getFaceInfo(fromFace).res;
                unsigned char* data = new unsigned char[f.res.size()*datasize];
                fromPtex->getData(fromFace, data, 0);

                for(size_t ii=0;ii<size_t(f.res.size());++ii)
                {
                    unsigned char cw = static_cast<unsigned char>(255);
                    unsigned char cb = static_cast<unsigned char>(0);
                    for(int c=0;c<numchan;++c)
                    {
                        if(white_faces[faceid])
                        {
                            data[c * f.res.size() + ii] = cw;
                        }
                        else
                        {
                            data[c * f.res.size() + ii] = cb;
                        }
                    }
                }
                w->writeFace(faceid, f, data, 0);
                delete [] data;
                
                copied++;

                continue;
            }
        }
    }
    
    delete [] errval;
    
    std::cout << "\n";
    if (copied>0) std::cout << "  Copied " << copied << " faces using a tolerance of " << matchDist << std::endl;
    
    std::cout << "  Adding meta data..." << std::endl;
    w->writeMeta(meta);
    
    std::cout << "  Closing file..." << std::endl;
    if (!w->close(ptexError))
    {
        std::cerr << ptexError << std::endl;
        return false;
    }
    
    time_t timer2;
    time(&timer2);
    double seconds = difftime(timer2,timer1);
    if (seconds<2)
        std::cout << "  Finished";
    else
        std::cout << "  Finished in " << seconds << " seconds";
        
    // int threads = omp_get_max_threads();
    // std::cout << " using " << threads << " thread" << ((threads>1)?"s":"") << "\n";
    
    return true;
}
   
void showUsage()
{
    std::cout << "Usage:\n";
    std::cout << "    ptxtransfer [options] <old.ptx> <new.ptx> <roots.obj>\n\n";
    std::cout << "Options:\n";
    std::cout << "    -d distance    maximum texel search distance\n";
    std::cout << "    -m distance    tolerance for direct face transfer\n";
    std::cout << "    -h             show help message\n";
    std::cout << "    -t num         set number of threads\n";
}

int main(int argc, char **argv)
{
    if (argc<3)
    {
        showUsage();
        return 1;
    }
    
    std::string input = "";
    std::string output = "";
    std::string roots = "";
    int numthreads = 1;
    float distance = -1;
    float match = -1;
    for (int i = 1; i < argc;)
    {
        std::string arg = argv[i++];
        
        if (arg[0] == '-')
        {
            if (arg == "-h" || arg == "-help")
            {
                showUsage();
                return 0;
            }
            else if (arg == "-t")
            {
                if (i >= argc) { showUsage(); return 1; }
                numthreads = atoi(argv[i++]);
            }
            else if (arg == "-d")
            {
                if (i >= argc) { showUsage(); return 1; }
                distance = atof(argv[i++]);
            }
            else if (arg == "-m")
            {
                if (i >= argc) { showUsage(); return 1; }
                match = atof(argv[i++]);
            }
            else
            {
                std::cerr << "Error: Invalid option: " << arg << "\n";
                return 1;
            }
        }
        else if (input == "") input = arg;
        else if (output == "") output = arg;
        else if (roots == "") roots = arg;
        else 
        {
            showUsage();
            return 0;
        }
    }
    
    if (input == "" || output == "" || roots == "")
    {
        showUsage();
        return 0;
    }
    
    // omp_set_num_threads(numthreads);
    transferPtex(input, output, roots, distance, match);
}
