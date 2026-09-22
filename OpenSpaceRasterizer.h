#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

class MaskedOcclusionCulling;

// Coverage-oriented companion to MaskedOcclusionCulling. It deliberately keeps
// the normal MOC occlusion API unchanged: triangles submitted here open pixels
// in a separate buffer instead of becoming occluders in the normal buffer.
class OpenSpaceRasterizer {
public:
    OpenSpaceRasterizer(unsigned width, unsigned height);
    ~OpenSpaceRasterizer();

    OpenSpaceRasterizer(const OpenSpaceRasterizer&) = delete;
    OpenSpaceRasterizer& operator=(const OpenSpaceRasterizer&) = delete;

    void ClearBuffer();

    // Vertices are (x,y,z,w) clip-space vertices, matching RenderTriangles().
    // Rasterized coverage is accumulated in the open-space buffer.
    MaskedOcclusionCulling::CullingResult RasterizeTriangles(
        const float* inVtx, const unsigned int* inTris, int nTris,
        const float* modelToClipMatrix = nullptr,
        MaskedOcclusionCulling::BackfaceWinding bfWinding = MaskedOcclusionCulling::BACKFACE_NONE,
        MaskedOcclusionCulling::ClipPlanes clipPlaneMask = MaskedOcclusionCulling::CLIP_PLANE_ALL,
        const MaskedOcclusionCulling::VertexLayout& vtxLayout =
            MaskedOcclusionCulling::VertexLayout(16, 4, 12));

    // Returns true if at least one pixel in the projected NDC rectangle has
    // been opened by empty-space geometry at a depth compatible with wmin.
    bool TestRect(float xmin, float ymin, float xmax, float ymax, float wmin) const;

    unsigned width() const { return width_; }
    unsigned height() const { return height_; }
    const std::vector<float>& depth() const { return openDepth_; }

private:
    void rebuildOpenDepth();

    MaskedOcclusionCulling* rasterizer_;
    unsigned width_;
    unsigned height_;
    std::vector<float> openDepth_;
};
