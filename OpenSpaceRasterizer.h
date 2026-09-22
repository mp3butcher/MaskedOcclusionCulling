#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include "MaskedOcclusionCulling.h"

// Conservative inverse-visibility buffer. Unlike RenderTriangles(), this class
// does not write MOC's occluder buffer: submitted triangles open coverage in a
// separate buffer, which is suitable for empty-space rasterization.
class OpenSpaceRasterizer {
public:
    OpenSpaceRasterizer(unsigned width, unsigned height);

    void ClearBuffer();

    MaskedOcclusionCulling::CullingResult RasterizeTriangles(
        const float* inVtx, const unsigned int* inTris, int nTris,
        const float* modelToClipMatrix = nullptr,
        MaskedOcclusionCulling::BackfaceWinding bfWinding = MaskedOcclusionCulling::BACKFACE_NONE,
        MaskedOcclusionCulling::ClipPlanes clipPlaneMask = MaskedOcclusionCulling::CLIP_PLANE_ALL,
        const MaskedOcclusionCulling::VertexLayout& vtxLayout =
            MaskedOcclusionCulling::VertexLayout(16, 4, 12));

    // Returns true when at least one open sample overlaps the NDC rectangle.
    bool TestRect(float xmin, float ymin, float xmax, float ymax, float wmin) const;

    unsigned width() const { return width_; }
    unsigned height() const { return height_; }
    const std::vector<float>& depth() const { return openDepth_; }

private:
    struct Vertex { float x, y, z, w; };
    void readVertex(const float* base, unsigned index,
                    const float* matrix,
                    const MaskedOcclusionCulling::VertexLayout& layout,
                    Vertex& out) const;
    void rasterizeTriangle(const Vertex& a, const Vertex& b, const Vertex& c);

    unsigned width_;
    unsigned height_;
    std::vector<float> openDepth_;
};
