//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_VERTEX_FORMAT_H
#define HAMO_LINUX_MC_VERTEX_FORMAT_H

namespace HAMO {

class VertexFormat {
   public:
    enum {
        MC_MAX_ATTRIBUTES = 16,
        MC_MAX_TCOORD_UNITS = 8,
        MC_MAX_COLOR_UNITS = 2,
    };

    enum AttributeUsage {
        MCAU_NONE,
        MCAU_POSITION,      // attr 0
        MCAU_NORMAL,        // attr 2
        MCAU_TANGENT,       // attr 14
        MCAU_BINORMAL,      // attr 15
        MCAU_TEXCOORD,      // attr 8-15
        MCAU_COLOR,         // attr 3-4
        MCAU_BLENDINDICES,  // attr 7
        MCAU_BLENDWEIGHT,   // attr 1
        MCAU_FOGCOORD,      // attr 5
        MCAU_PSIZE,         // attr 6
    };

    enum AttributeType {
        MCAT_NONE,
        MCAT_FLOAT1,
        MCAT_FLOAT2,
        MCAT_FLOAT3,
        MCAT_FLOAT4,
        MCAT_UBYTE4,
        MCAT_SHORT1,
        MCAT_SHORT2,
        MCAT_SHORT4,
        MCAT_QUANTITY
    };

   public:
    VertexFormat();

    ~VertexFormat();

   public:
    int GetIndex(AttributeUsage usage, unsigned int usageIndex = 0) const;

    void AppendAttribute(AttributeUsage usage, AttributeType type,
                         unsigned int usageInde);

    void GetAttribute(int attribute, unsigned int &streamIndex,
                      unsigned int &offset, AttributeType &type,
                      AttributeUsage &usage, unsigned int &usageIndex) const;

    void SetAttribute(int attribute, unsigned int streamIndex,
                      unsigned int offset, AttributeType type,
                      AttributeUsage usage, unsigned int usageIndex);

    int GetStride() const;

    int GetNumAttributes() const;

    unsigned int GetStreamIndex(int attribute) const;

    unsigned int GetOffset(int attribute) const;

    AttributeType GetAttributeType(int attribute) const;

    AttributeUsage GetAttributeUsage(int attribute) const;

    unsigned int GetUsageIndex(int attribute) const;

   public:
    static int GetComponentSize(AttributeType type);

    static int GetNumComponents(AttributeType type);

    static int GetTypeSize(AttributeType type);

   protected:
    struct Element {
        unsigned int stream_index;
        unsigned int offset;
        AttributeType type;
        AttributeUsage usage;
        unsigned int usage_index;
    };

   private:
    int stride_;
    int num_attributes_;
    Element elements_[MC_MAX_ATTRIBUTES];

    static int type_size_[MCAT_QUANTITY];
    static int num_components_[MCAT_QUANTITY];
    static int component_size_[MCAT_QUANTITY];
};

}  // namespace HAMO

#endif  // HAMO_LINUX_MC_VERTEX_FORMAT_H
