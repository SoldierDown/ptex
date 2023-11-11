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
    PtexPtr<PtexTexture> tx(PtexTexture::open("planeE.ptx", error));
    if (!tx) {
        std::cerr << error.c_str() << std::endl;
        return 0;
    }
    PtexPtr<PtexMetaData> meta(tx->getMetaData());
    std::vector<int32_t> faceVertCounts;
    std::vector<int32_t> faceVertIndices;
    std::vector<float> vertPositions;
    // get meta info: face vert counts
    {
        int count;
        const int32_t* val;
        meta->getValue("PtexFaceVertCounts", val, count);
        for(int i=0;i<count;++i)
        {
            faceVertCounts.push_back(val[i]);
        }
    }
    // vert positions
    {
        int count;
        const float* val;
        meta->getValue("PtexVertPositions", val, count);
        for(int i=0;i<count;++i)
        {
            vertPositions.push_back(val[i]);
        }
    }
    // face vert indices
    {
        int count;
        const int32_t* val;
        meta->getValue("PtexFaceVertIndices", val, count);
        for(int i=0;i<count;++i)
        {
            faceVertIndices.push_back(val[i]);
        }
    }


    int nfaces = faceVertCounts.size();
    Ptex::DataType dt = Ptex::dt_uint16;
    float ptexOne = Ptex::OneValue(dt);
    typedef uint16_t Dtype;
    int alpha = -1;
    int nchan = 3;
    PtexWriter* w = PtexWriter::open("planeEE.ptx", Ptex::mt_quad, dt, nchan, alpha, nfaces, error);
    if (!w) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    int size = 0;
    for (int i = 0; i < nfaces; i++)
        size = std::max(size, faceVertCounts[i]);
    size *= Ptex::DataSize(dt) * nchan;
    std::cout << "size: " << size << std::endl;
    void* buff = malloc(size);
    for (int i = 0; i < nfaces; i++)
    {
        memset(buff, 0, size);
        Dtype* fbuff = (Dtype*)buff;
        int ures = 2, vres = 2;
        for (int v = 0; v < vres; v++) {
            for (int u = 0; u < ures; u++) {
                float c = (u ^ v) & 1;
                fbuff[(v*ures+u)*nchan] = u/float(ures-1) * ptexOne;
                fbuff[(v*ures+u)*nchan+1] = v/float(vres-1) * ptexOne;
                fbuff[(v*ures+u)*nchan+2] = c * ptexOne;
            }
        }
        static int32_t adjedges[] = {tx->getFaceInfo(i).adjedge(0), tx->getFaceInfo(i).adjedge(1), tx->getFaceInfo(i).adjedge(2), tx->getFaceInfo(i).adjedge(3)};
        static int32_t adjfaces[] = {tx->getFaceInfo(i).adjface(0), tx->getFaceInfo(i).adjface(1), tx->getFaceInfo(i).adjface(2), tx->getFaceInfo(i).adjface(3)};
        w->writeFace(i, Ptex::FaceInfo(Ptex::Res(1, 1), adjfaces, adjedges), buff);
    }
    std::cout << "done" << std::endl;
    if (!w->close(error)) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    free(buff);
/*
    int nKeys = meta->numKeys();
    std::cout << "nKeys: " << nKeys << std::endl;
    for(int kid=0;kid<nKeys;++kid)
    {
        const char* key;
        MetaDataType type;
        meta->getKey(kid, key, type);
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
*/
    return 0;
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

    int nfaces = sizeof(res)/sizeof(res[0]);
    Ptex::DataType dt = Ptex::dt_uint16;
    float ptexOne = Ptex::OneValue(dt);
    typedef uint16_t Dtype;
    int alpha = -1;
    int nchan = 3;

    Ptex::String error;
    PtexWriter* w =
        PtexWriter::open("planeE.ptx", Ptex::mt_quad, dt, nchan, alpha, nfaces, error);
    if (!w) {
        std::cerr << error.c_str() << std::endl;
        return 1;
    }
    int size = 0;
    for (int i = 0; i < nfaces; i++)
        size = std::max(size, res[i].size());
    size *= Ptex::DataSize(dt) * nchan;

    void* buff = malloc(size);
    for (int i = 0; i < nfaces; i++)
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
    w = PtexWriter::edit("planeE.ptx", true, Ptex::mt_quad, dt, nchan, alpha, nfaces, error);
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

    w = PtexWriter::edit("planeE.ptx", false, Ptex::mt_quad, dt, nchan, alpha, nfaces, error);
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
