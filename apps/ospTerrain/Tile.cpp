#include "Tile.h"
#include "ospcommon/utility/multidim_index_sequence.h"
#include "Lerc_c_api.h"

typedef unsigned char Byte;

Tile::Tile() {
  this->elevation = nullptr;
  this->imagedata = nullptr;
}


Tile::~Tile() {
  if (this->elevation != nullptr) {
    delete[] this->elevation;
  }
  if (this->imagedata != nullptr) {
    delete[] this->imagedata;
  }
}


void Tile::readURL(std::string url, TileType type)
{
  lerc_status hr(0);

  std::ifstream filestream(url, std::ios::binary);
  if (!filestream.is_open()) {
    std::cout << "failed to open " << url << '\n';
  } else {
    filestream.seekg(0, std::ios::end);
    std::ifstream::pos_type pos = filestream.tellg();
    int fileSize = pos;
    char *buffer = new char[fileSize];
    filestream.seekg(0, std::ios::beg);
    filestream.read(buffer, fileSize);
    filestream.close();
    unsigned char *finalBuffer = reinterpret_cast<unsigned char *>(buffer);

    if (type == ELEVATION) {
        uint32_t infoArr[10];
        double dataRangeArr[3];
        hr = lerc_getBlobInfo(
            finalBuffer, (unsigned int)fileSize, infoArr, dataRangeArr, 10, 3);
        if (hr) {
          std::cout << "lerc_getBlobInfo(...) failed" << std::endl;
          return;
        } 

          int version = infoArr[0];
          unsigned int datatype = infoArr[1];
          int nDim = infoArr[2];
          this->pixelSize = {infoArr[3], infoArr[4]};
          int nBands = infoArr[5];
          this->dataRange = {dataRangeArr[0], dataRangeArr[1]};
          int sizeOfIncomintDatatype = datatype == 0
              ? sizeof(char)
              : datatype == 1 ? sizeof(unsigned char)
              : datatype == 2 ? sizeof(short)
              : datatype == 3 ? sizeof(unsigned short)
              : datatype == 4 ? sizeof(int)
              : datatype == 5 ? sizeof(unsigned int)
              : datatype == 6 ? sizeof(float)
              : sizeof(double);

          // char = 0, uchar = 1, short = 2, ushort = 3, int = 4, uint = 5, float =
          // 6, double = 7

          int imgPixelCount = this->pixelSize.x * this->pixelSize.y;
          int imgSize = imgPixelCount * nDim * nBands
              * sizeOfIncomintDatatype;

          unsigned char *pValidBytes = new unsigned char[imgPixelCount];

          std::string resultMsg = "ok";
          // check if elevation or RGB tile
          lerc_status readStatus;
          if (datatype > 1) {
            switch (datatype) {
                default: {
              this->elevation = new double[imgPixelCount * nDim * nBands];
                  readStatus = lerc_decodeToDouble(finalBuffer,
                      (unsigned int)fileSize,
                      pValidBytes,
                      nDim,
                      this->pixelSize.x,
                      this->pixelSize.y,
                      nBands,
                      this->elevation);
                  break;
                }
            }
        
          } else {
            this->imagedata = new unsigned char[imgSize];
      
            readStatus = lerc_decode(finalBuffer,
                (unsigned int)fileSize,
                pValidBytes,
                nDim,
                this->pixelSize.x,
                this->pixelSize.y,
                nBands,
                datatype,
                (void *)this->imagedata);
          }

          if (0 != readStatus) {
            resultMsg = "lerc_decode failed";
          }

          delete[] pValidBytes;

    } else {

      const char *subsampName[TJ_NUMSAMP] = {
          "4:4:4", "4:2:2", "4:2:0", "Grayscale", "4:4:0", "4:1:1"};
      const char *colorspaceName[TJ_NUMCS] = {
          "RGB", "YCbCr", "GRAY", "CMYK", "YCCK"};
     
      unsigned char *imgBuf = NULL, *jpegBuf = NULL;
      tjhandle tjInstance = NULL;
      int inSubsamp, inColorspace;
      
      int width;
      int height;
      int jpegSubsamp;

      tjhandle _jpegDecompressor = tjInitDecompress();
     tjDecompressHeader2(_jpegDecompressor,
          finalBuffer,
          fileSize,
          &width,
          &height,
          &jpegSubsamp);

      this->imagedata = new unsigned char[width * height * 3];
     tjDecompress2(_jpegDecompressor,
         finalBuffer,
         fileSize,
         this->imagedata,
         width,
         0 /*pitch*/,
         height,
         TJPF_RGB,
         TJFLAG_FASTDCT);

      tjDestroy(_jpegDecompressor);

      this->pixelSize = {(unsigned int)width, (unsigned int)height};
    }

    delete[] buffer;
  }
}

double* Tile::getElevation()
{
  return this->elevation;
}

unsigned char* Tile::getImage()
{
  return this->imagedata;
}