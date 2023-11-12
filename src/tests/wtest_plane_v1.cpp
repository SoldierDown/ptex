#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "Ptexture.h"
#include "PtexHalf.h"
#include <string.h>
#include <vector>
using namespace Ptex;

void writeMeta(PtexWriter* w,
               const char* sval, double* dvals, int ndvals, int16_t* ivals, int nivals,
               const char* xval)
{
    if (sval) w->writeMeta("sval", sval);
    if (dvals) w->writeMeta("dvals", dvals, ndvals);
    if (ivals) w->writeMeta("ivals", ivals, nivals);
    if (xval) w->writeMeta("xval", xval);
}


bool checkMeta(const char* path,
               const char* sval, double* dvals, int ndvals, int16_t* ivals, int nivals,
               const char* xval)
{
    Ptex::String error;
    PtexPtr<PtexTexture> tx(PtexTexture::open(path, error));
    if (!tx) {
        std::cerr << error.c_str() << std::endl;
        return 0;
    }
    PtexPtr<PtexMetaData> meta(tx->getMetaData());

    const char* f_sval;
    meta->getValue("sval", f_sval);

    const double* f_dvals;
    int f_ndvals;
    meta->getValue("dvals", f_dvals, f_ndvals);

    const int16_t* f_ivals;
    int f_nivals;
    meta->getValue("ivals", f_ivals, f_nivals);

    const char* f_xval;
    meta->getValue("xval", f_xval);

    bool ok = ((!sval || 0==strcmp(sval, f_sval)) &&
               (!ndvals || (ndvals == f_ndvals &&
                            0==memcmp(dvals, f_dvals,
                                      ndvals * sizeof(dvals[0])))) &&
               (!nivals || (nivals == f_nivals &&
                            0==memcmp(ivals, f_ivals,
                                      nivals*sizeof(ivals[0])))) &&
               (!xval || 0==strcmp(xval, f_xval)));
    if (!ok) {
        std::cerr << "Meta data readback failed" << std::endl;
        return 0;
    }
    return 1;
}


int main(int /*argc*/, char** /*argv*/)
{
    Ptex::String error;
    PtexPtr<PtexTexture> tx(PtexTexture::open("./ptx_files/mask_black_64.ptx", error));
    if (!tx) {
        std::cerr << error.c_str() << std::endl;
        return 0;
    }
    Ptex::ResFaceData dataa;
    tx->putData(0, 0, dataa);
    PtexPtr<PtexMetaData> meta(tx->getMetaData());
    std::vector<int32_t> faceVertCounts;
    std::vector<int32_t> faceVertIndices;
    std::vector<float> vertPositions;
    // get meta info: face vert counts
    int fvc_count;
    const int32_t* fvc_val;
    meta->getValue("PtexFaceVertCounts", fvc_val, fvc_count);
    for(int i=0;i<fvc_count;++i)
    {
        faceVertCounts.push_back(fvc_val[i]);
    }
    std::cout << "reading " << faceVertCounts.size() << " faces" << std::endl;
    // vert positions
    int vp_count;
    const float* vp_val;
    meta->getValue("PtexVertPositions", vp_val, vp_count); 
    for(int i=0;i<vp_count;++i)
    {
        vertPositions.push_back(vp_val[i]);
    }
    std::cout << "reading " << vertPositions.size() << " vertices" << std::endl;
    // face vert indices
    int fvi_count;
    const int32_t* fvi_val;
    meta->getValue("PtexFaceVertIndices", fvi_val, fvi_count);
    for(int i=0;i<fvi_count;++i)
    {
        faceVertIndices.push_back(fvi_val[i]);
    }
    std::cout << "reading " << faceVertIndices.size() << " quad vertices" << std::endl;
    int numFaces = faceVertCounts.size();
    Ptex::DataType dt = Ptex::dt_uint16;
    float ptexOne = Ptex::OneValue(dt);
    typedef uint8_t Dtype;
    int alpha = 1;
    int nchan = 3;
    PtexWriter* w = PtexWriter::open("./ptx_files/mask_black_64_write.ptx", Ptex::mt_quad, dt, nchan, alpha, numFaces, error);
    if (!w) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    int size = faceVertCounts[0] * Ptex::DataSize(dt) * nchan;
    std::cout << "size: " << size << std::endl;
    void* buff = malloc(size);
    for (int i = 0; i < numFaces; i++)
    {
        memset(buff, 0, size);
        Dtype* fbuff = (Dtype*)buff;
        int ures = 8, vres = 8;
        for (int v = 0; v < vres; v++) {
            for (int u = 0; u < ures; u++) {
                if(i < numFaces/2)
                {
                    fbuff[(v*ures+u)*nchan] =   ptexOne;
                    fbuff[(v*ures+u)*nchan+1] = ptexOne;
                    fbuff[(v*ures+u)*nchan+2] = ptexOne;
                }
                else 
                {
                    fbuff[(v*ures+u)*nchan] = 0;
                    fbuff[(v*ures+u)*nchan+1] = 0;
                    fbuff[(v*ures+u)*nchan+2] = 0;
                }
                    fbuff[(v*ures+u)*nchan] =   ptexOne;
                    fbuff[(v*ures+u)*nchan+1] = ptexOne;
                    fbuff[(v*ures+u)*nchan+2] = ptexOne;
            }
        }
        int adjedges[] = {tx->getFaceInfo(i).adjedge(0), tx->getFaceInfo(i).adjedge(1), tx->getFaceInfo(i).adjedge(2), tx->getFaceInfo(i).adjedge(3)};
        int adjfaces[] = {tx->getFaceInfo(i).adjface(0), tx->getFaceInfo(i).adjface(1), tx->getFaceInfo(i).adjface(2), tx->getFaceInfo(i).adjface(3)};
        std::cout << "adjfaces: " << tx->getFaceInfo(i).adjface(0) << ", " << tx->getFaceInfo(i).adjface(1) << ", " << tx->getFaceInfo(i).adjface(2) << ", " << tx->getFaceInfo(i).adjface(3) 
                << ", adjedges: " << tx->getFaceInfo(i).adjedge(0) << ", " << tx->getFaceInfo(i).adjedge(1) << ", " << tx->getFaceInfo(i).adjedge(2) << ", " << tx->getFaceInfo(i).adjedge(3) << std::endl;
        std::cout << "adjfaces: " << adjfaces[0] << ", " << adjfaces[1] << ", " << adjfaces[2] << ", " << adjfaces[3] << ", adjedges: " << adjedges[0] << ", "<< adjedges[1] << ", "<< adjedges[2] << ", " << adjedges[3] << std::endl;
        w->writeFace(i, Ptex::FaceInfo(Ptex::Res(3, 3), adjfaces, adjedges), buff);
    }
    if(vp_val) 
    {
        std::cout << "writing PtexVertPositions" << std::endl;
        w->writeMeta("PtexVertPositions", vp_val, vp_count);
        std::cout << "DONE: writing PtexVertPositions" << std::endl;
    }
    if(fvi_val) 
    {
        std::cout << "writing PtexFaceVertIndices" << std::endl;
        w->writeMeta("PtexFaceVertIndices", fvi_val, fvi_count);
        std::cout << "DONE: writing PtexVertPositions" << std::endl;
    }
    if(fvc_val) 
    {
        std::cout << "writing PtexFaceVertCounts" << std::endl;
        w->writeMeta("PtexFaceVertCounts", fvc_val, fvc_count);
        std::cout << "DONE: writing PtexVertPositions" << std::endl;
    }
    if (!w->close(error)) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    w->release();
    std::cout << "close done" << std::endl;
    int nKeys = meta->numKeys();
    std::cout << "nKeys: " << nKeys << std::endl;
    for(int kid=0;kid<nKeys;++kid)
    {
        const char* key;
        MetaDataType type;
        meta->getKey(kid, key, type);
        std::cout << "key: " << key <<  std::endl;
        std::cout << "type: " << type << std::endl;
        int count = 0;
        switch (type)
        {
            case mdt_string:
            {
                const char* sval;
                meta->getValue(key, sval);
                std::cout << key << " | " << type << " | " << sval << " | " << count <<std::endl;
            }
            break;
            case mdt_int8:
            {
                const int8_t* sval;
                meta->getValue(key, sval, count);
                std::cout << key << " | " << type << " | " << count <<std::endl;
                std::cout << "data size: " << count << std::endl;
                for(int i=0;i<count;++i)
                {
                    // std::cout << sval[i] << std::endl;
                }
            }
            break;
            case mdt_int16:
            {
                const int16_t* sval;
                meta->getValue(key, sval, count);
                std::cout << key << " | " << type << " | " << count <<std::endl;
                std::cout << "data size: " << count << std::endl;
                for(int i=0;i<count;++i)
                {
                    // std::cout << sval[i] << std::endl;
                }
            }
            break;
            case mdt_int32:
            {
                const int32_t* sval;
                meta->getValue(key, sval, count);
                std::cout << key << " | " << type << " | " << count <<std::endl;
                std::cout << "data size: " << count << std::endl;
                for(int i=0;i<count;++i)
                {
                    // std::cout << sval[i] << std::endl;
                }
            }
            break;
            case mdt_float:
            {
                const float* sval;
                meta->getValue(key, sval, count);
                std::cout << key << " | " << type << " | " << count <<std::endl;
                std::cout << "data size: " << count << std::endl;
                for(int i=0;i<count;++i)
                {
                    // std::cout << sval[i] << std::endl;
                }
            }
            break;
            case mdt_double:
            {
                const double* sval;
                meta->getValue(key, sval, count);
                std::cout << key << " | " << type << " | " << count <<std::endl;
                std::cout << "data size: " << count << std::endl;
                for(int i=0;i<count;++i)
                {
                    // std::cout << sval[i] << std::endl;
                }
            }
            break;
            default:
            break;
        }   
    }
    return 0;

    // write things like below and add vertex position info, face vertex info etc
/*
    
    static Ptex::Res res[] = { Ptex::Res(8,7),
                               Ptex::Res(0x0201),
                               Ptex::Res(3,1),
                               Ptex::Res(0x0405),
                               Ptex::Res(9,8),
                               Ptex::Res(0x0402),
                               Ptex::Res(6,2),
                               Ptex::Res(0x0407),
                               Ptex::Res(2,1)};
    static int adjedges[][4] = {{ 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 },
                                { 2, 3, 0, 1 }};
    static int adjfaces[][4] ={{ 3, 1, -1, -1 },
                               { 4, 2, -1, 0 },
                               { 5, -1, -1, 1 },
                               { 6, 4, 0, -1 },
                               { 7, 5, 1, 3 },
                               { 8, -1, 2, 4 },
                               { -1, 7, 3, -1 },
                               { -1, 8, 4, 6 },
                               { -1, -1, 5, 7 }};

    int numFaces = sizeof(res)/sizeof(res[0]);
    Ptex::DataType dt = Ptex::dt_uint16;
    float ptexOne = Ptex::OneValue(dt);
    typedef uint16_t Dtype;
    int alpha = -1;
    int nchan = 3;

    Ptex::String error;
    PtexWriter* w =
        PtexWriter::open("planeE.ptx", Ptex::mt_quad, dt, nchan, alpha, numFaces, error);
    if (!w) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    int size = 0;
    for (int i = 0; i < numFaces; i++)
        size = std::max(size, res[i].size());
    size *= Ptex::DataSize(dt) * nchan;

    void* buff = malloc(size);
    for (int i = 0; i < numFaces; i++)
    {
        memset(buff, 0, size);
        Dtype* fbuff = (Dtype*)buff;
        int ures = res[i].u(), vres = res[i].v();
        for (int v = 0; v < vres; v++) {
            for (int u = 0; u < ures; u++) {
                float c = (u ^ v) & 1;
                fbuff[(v*ures+u)*nchan] = u/float(ures-1) * ptexOne;
                fbuff[(v*ures+u)*nchan+1] = v/float(vres-1) * ptexOne;
                fbuff[(v*ures+u)*nchan+2] = c * ptexOne;
            }
        }

        w->writeFace(i, Ptex::FaceInfo(res[i], adjfaces[i], adjedges[i]), buff);
    }
    free(buff);

    const char* sval = "a str val";
    int ndvals = 3;
    double dvals_buff[3] = { 1.1,2.2,3.3 };
    double* dvals = dvals_buff;
    int nivals = 4;
    int16_t ivals[4] = { 2, 4, 6, 8 };
    const char* xval = 0;

    writeMeta(w, sval, dvals, ndvals, ivals, nivals, xval);
    if (!w->close(error)) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    w->release();
    if (!checkMeta("planeE.ptx", sval, dvals, ndvals, ivals, nivals, xval))
        return 1;

    // add some incremental edits
    w = PtexWriter::edit("planeE.ptx", true, Ptex::mt_quad, dt, nchan, alpha, numFaces, error);
    sval = "a string value";
    dvals[2] = 0;
    writeMeta(w, sval, dvals, ndvals, 0, 0, 0);

    if (!w->close(error)) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    w->release();
    if (!checkMeta("planeE.ptx", sval, dvals, ndvals, ivals, nivals, xval))
        return 1;

    // add some non-incremental edits, including some large meta data
    ndvals = 500;
    dvals = (double*)malloc(ndvals * sizeof(dvals[0]));
    for (int i = 0; i < ndvals; i++) dvals[i] = i;

    w = PtexWriter::edit("planeE.ptx", false, Ptex::mt_quad, dt, nchan, alpha, numFaces, error);
    xval = "another string value";
    writeMeta(w, 0, dvals, ndvals, 0, 0, xval);
    if (!w->close(error)) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    w->release();
    if (!checkMeta("planeE.ptx", sval, dvals, ndvals, ivals, nivals, xval))
        return 1;
    free(dvals);

    return 0;
*/
}
